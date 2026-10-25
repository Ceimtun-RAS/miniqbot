"""
MiniQbotExport: dump the raw data of the design open in Autodesk Fusion.

This is the FIRST half of the Fusion -> URDF pipeline:

    Fusion (this script)                 Linux (build_urdf.py)
    -------------------------------      ----------------------------------
    one mesh per body (STL)        --->  groups parts into links,
    masses and inertias                  converts to ROS axes and meters,
    joints (axes, origins)               writes the URDF and the link meshes

This script makes NO decisions about the robot: it only copies what Fusion
knows. All robot-specific logic (which part belongs to which link, how the
knee linkage is handled) lives in build_urdf.py and robot_config.yaml,
which can be tested without Fusion.

What it does NOT do:
  It never modifies the design. It only reads it and writes files outside Fusion.

Output:
  C:\\Users\\<user>\\Downloads\\miniqbot_export\\
    export.json      parts, bodies and joints
    meshes\\*.stl     one mesh per body, in assembly coordinates

Units (raw Fusion API units, not converted):
  lengths in cm, masses in kg, inertias in kg*cm^2 about the assembly ORIGIN
  (not about the center of mass), angles in radians.
"""

import json
import os
import re
import struct
import traceback

import adsk.core
import adsk.fusion

FORMAT_VERSION = 1

JOINT_TYPES = {
    0: "rigid",
    1: "revolute",
    2: "slider",
    3: "cylindrical",
    4: "pin_slot",
    5: "planar",
    6: "ball",
}


def occurrence_name(occ):
    """Full path of an occurrence (e.g. "base_link:1+Raspberry...:1"); None = root."""
    if occ is None:
        return None
    try:
        return occ.fullPathName
    except Exception:
        return occ.name


def safe_file_name(text):
    """Turn a Fusion name into a safe file name."""
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", text).strip("_")


def point(p):
    return None if p is None else [p.x, p.y, p.z]


def vector(v):
    return None if v is None else [v.x, v.y, v.z]


def joint_origin(geometry):
    """Origin of a JointGeometry or a JointOrigin, in assembly coordinates."""
    if geometry is None:
        return None
    try:
        if geometry.origin is not None and hasattr(geometry.origin, "x"):
            return point(geometry.origin)
    except Exception:
        pass
    try:
        return point(geometry.geometry.origin)
    except Exception:
        return None


def write_binary_stl(path, coordinates, indices):
    """Write a triangle mesh as binary STL. Coordinates are written as-is (cm)."""
    triangle_count = len(indices) // 3
    with open(path, "wb") as stl:
        stl.write(b"MiniQbotExport (cm)".ljust(80, b" "))
        stl.write(struct.pack("<I", triangle_count))
        for t in range(triangle_count):
            vertices = []
            for k in range(3):
                i = indices[3 * t + k]
                vertices.append(coordinates[3 * i:3 * i + 3])
            (ax, ay, az), (bx, by, bz), (cx, cy, cz) = vertices
            ux, uy, uz = bx - ax, by - ay, bz - az
            vx, vy, vz = cx - ax, cy - ay, cz - az
            nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            length = (nx * nx + ny * ny + nz * nz) ** 0.5 or 1.0
            stl.write(struct.pack("<3f", nx / length, ny / length, nz / length))
            for v in vertices:
                stl.write(struct.pack("<3f", *v))
            stl.write(b"\x00\x00")
    return triangle_count


def export_body(body, mesh_folder, file_prefix):
    data = {"name": body.name}
    try:
        data["visible"] = body.isVisible
        data["solid"] = body.isSolid
        box = body.boundingBox
        data["bounding_box_cm"] = [point(box.minPoint), point(box.maxPoint)]
    except Exception as error:
        data["bounding_box_error"] = str(error)
    try:
        props = body.physicalProperties
        data["mass_kg"] = props.mass
        data["volume_cm3"] = props.volume
        data["center_of_mass_cm"] = point(props.centerOfMass)
        ok, xx, yy, zz, xy, yz, xz = props.getXYZMomentsOfInertia()
        if ok:
            data["inertia_about_origin_kgcm2"] = {"xx": xx, "yy": yy, "zz": zz,
                                                  "xy": xy, "yz": yz, "xz": xz}
    except Exception as error:
        data["physical_properties_error"] = str(error)
    try:
        material = body.material
        data["material"] = material.name if material else None
    except Exception:
        data["material"] = None
    try:
        calculator = body.meshManager.createMeshCalculator()
        calculator.setQuality(adsk.fusion.TriangleMeshQualityOptions.NormalQualityTriangleMesh)
        mesh = calculator.calculate()
        file_name = file_prefix + ".stl"
        data["triangles"] = write_binary_stl(
            os.path.join(mesh_folder, file_name),
            mesh.nodeCoordinatesAsDouble, mesh.nodeIndices)
        data["mesh"] = "meshes/" + file_name
    except Exception as error:
        data["mesh_error"] = str(error)
    return data


def export_motion(motion):
    data = {}
    try:
        data["type"] = JOINT_TYPES.get(motion.jointType, str(motion.jointType))
    except Exception:
        return data
    revolute = adsk.fusion.RevoluteJointMotion.cast(motion)
    if revolute:
        data["axis"] = vector(revolute.rotationAxisVector)
        data["angle_rad"] = revolute.rotationValue
        limits = revolute.rotationLimits
        data["limits_rad"] = {
            "min": limits.minimumValue if limits.isMinimumValueEnabled else None,
            "max": limits.maximumValue if limits.isMaximumValueEnabled else None,
        }
    slider = adsk.fusion.SliderJointMotion.cast(motion)
    if slider:
        data["axis"] = vector(slider.slideDirectionVector)
    cylindrical = adsk.fusion.CylindricalJointMotion.cast(motion)
    if cylindrical:
        data["axis"] = vector(cylindrical.rotationAxisVector)
    return data


def export_joint(joint):
    data = {"name": joint.name}
    for key, getter in (
        ("occurrence_one", lambda: occurrence_name(joint.occurrenceOne)),
        ("occurrence_two", lambda: occurrence_name(joint.occurrenceTwo)),
        ("suppressed", lambda: joint.isSuppressed),
        ("motion", lambda: export_motion(joint.jointMotion)),
        ("origin_cm", lambda: joint_origin(joint.geometryOrOriginOne)),
        ("origin_two_cm", lambda: joint_origin(joint.geometryOrOriginTwo)),
    ):
        try:
            data[key] = getter()
        except Exception as error:
            data[key] = None
            data.setdefault("errors", []).append(key + ": " + str(error))
    return data


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No design is open. Open the robot assembly and run again.")
            return
        root = design.rootComponent

        output_folder = os.path.join(os.path.expanduser("~"), "Downloads", "miniqbot_export")
        mesh_folder = os.path.join(output_folder, "meshes")
        os.makedirs(mesh_folder, exist_ok=True)
        for old_file in os.listdir(mesh_folder):
            if old_file.endswith(".stl"):
                os.remove(os.path.join(mesh_folder, old_file))

        result = {
            "format_version": FORMAT_VERSION,
            "document": app.activeDocument.name,
            "units": {"length": "cm", "mass": "kg", "inertia": "kg*cm^2 about the origin",
                      "angle": "rad"},
            "occurrences": [],
            "joints": [],
            "rigid_groups": [],
        }

        for index, occ in enumerate(root.allOccurrences):
            entry = {
                "name": occurrence_name(occ),
                "component": occ.component.name,
                "light_bulb_on": occ.isLightBulbOn,
                "visible": occ.isVisible,
                "bodies": [],
            }
            for body_index, body in enumerate(occ.bRepBodies):
                prefix = "{:03d}_{}_{}".format(index, safe_file_name(occ.name), body_index)
                entry["bodies"].append(export_body(body, mesh_folder, prefix))
            result["occurrences"].append(entry)

        result["joints"] = [export_joint(j) for j in root.allJoints]
        for group in root.allRigidGroups:
            result["rigid_groups"].append({
                "name": group.name,
                "occurrences": [occurrence_name(o) for o in group.occurrences],
            })

        with open(os.path.join(output_folder, "export.json"), "w", encoding="utf-8") as out:
            json.dump(result, out, ensure_ascii=False, indent=2)

        body_count = sum(len(o["bodies"]) for o in result["occurrences"])
        ui.messageBox(
            "Export finished (the design was not modified).\n\n"
            "{} occurrences, {} bodies, {} joints\n\n{}".format(
                len(result["occurrences"]), body_count, len(result["joints"]), output_folder))
    except Exception:
        ui.messageBox("Export failed:\n\n" + traceback.format_exc())
