#!/usr/bin/env python3
"""
build_urdf.py: turn the raw Fusion export into the robot description.

This is the SECOND half of the Fusion -> URDF pipeline:

    Fusion (MiniQbotExport)              Linux (this script)
    -------------------------------      ----------------------------------
    one mesh per body (STL)        --->  groups parts into links,
    masses and inertias                  converts to ROS axes and meters,
    joints (axes, origins)               writes the URDF and the link meshes

All robot-specific decisions come from robot_config.yaml. This script only
applies them and checks that the CAD still matches them, so it fails with a
clear message instead of writing a broken URDF when the CAD changes.

Output (inside the miniqbot_description package):
    urdf/<robot_name>.urdf   links and joints (GENERATED, do not edit by hand)
    meshes/<link>.stl        one mesh per link, in the link frame, meters

Usage:
    python3 tools/fusion_to_urdf/build_urdf.py
    python3 tools/fusion_to_urdf/build_urdf.py --export <dir> --output <package dir>

Conventions used for every generated frame:
    - All link frames are parallel to base_link (no rotation), placed at the
      joint that moves the link. Angle 0 of every joint is the CAD pose.
    - Inertia tensors follow the URDF convention (ixy = -integral(x*y dm)),
      which is also what Fusion returns.
"""

import argparse
import fnmatch
import json
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np
import yaml

SUPPORTED_EXPORT_FORMAT = 1
SUPPORTED_CONFIG_FORMAT = 1

CM_TO_M = 0.01
KGCM2_TO_KGM2 = 1e-4

# Order of the segments in a leg, from the body outwards.
SEGMENTS = ("hip", "thigh", "knee")
# Link moved by each segment's joint.
SEGMENT_LINK = {"hip": "hip_link", "thigh": "thigh_link", "knee": "shin_link"}
SEGMENT_JOINT = {"hip": "hip_joint", "thigh": "thigh_joint", "knee": "knee_joint"}
SEGMENT_COLOR = {"hip": "hip", "thigh": "thigh", "knee": "shin"}

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_EXPORT = os.path.expanduser("~/Downloads/miniqbot_export")
DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_config.yaml")
DEFAULT_OUTPUT = os.path.join(REPO_ROOT, "src", "miniqbot_description")


class BuildError(Exception):
    """The export does not match robot_config.yaml. The message says how to fix it."""


# --------------------------------------------------------------------------
# STL files
# --------------------------------------------------------------------------

STL_RECORD = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (9,)), ("attribute", "<u2")])


def read_stl(path):
    """Read a binary STL file. Returns an (n, 3, 3) float array of triangles."""
    with open(path, "rb") as stl:
        raw = stl.read()
    count = int.from_bytes(raw[80:84], "little")
    if len(raw) != 84 + count * STL_RECORD.itemsize:
        raise BuildError("{} is not a valid binary STL file".format(path))
    records = np.frombuffer(raw, dtype=STL_RECORD, count=count, offset=84)
    return records["vertices"].reshape(count, 3, 3).astype(float)


def write_stl(path, triangles, header):
    """Write an (n, 3, 3) array of triangles as binary STL."""
    triangles = np.asarray(triangles, dtype=float).reshape(-1, 3, 3)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    lengths[lengths == 0] = 1.0
    records = np.zeros(len(triangles), dtype=STL_RECORD)
    records["normal"] = normals / lengths[:, None]
    records["vertices"] = triangles.reshape(-1, 9)
    with open(path, "wb") as stl:
        stl.write(header.encode("ascii")[:80].ljust(80, b" "))
        stl.write(len(triangles).to_bytes(4, "little"))
        stl.write(records.tobytes())


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------

def axis_mapping(mapping):
    """Rotation matrix R so that v_ros = R @ v_fusion, from e.g. {x: +z, y: +x, z: +y}."""
    rotation = np.zeros((3, 3))
    for row, ros_axis in enumerate("xyz"):
        spec = str(mapping[ros_axis]).strip()
        sign = -1.0 if spec.startswith("-") else 1.0
        rotation[row, "xyz".index(spec.lstrip("+-"))] = sign
    if not np.isclose(np.linalg.det(rotation), 1.0):
        raise BuildError("fusion_to_ros is not a rotation (it mirrors the robot): {}".format(mapping))
    return rotation


def clean_axis(axis, snap_tolerance_deg, normalize_sign):
    """Normalize a joint axis, snap it to a main axis if close, optionally make it positive."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    main = int(np.argmax(np.abs(axis)))
    angle = math.degrees(math.acos(min(1.0, abs(axis[main]))))
    if angle <= snap_tolerance_deg:
        snapped = np.zeros(3)
        snapped[main] = math.copysign(1.0, axis[main])
        axis = snapped
    if normalize_sign and axis[main] < 0:
        axis = -axis
    return axis, angle


def inertia_matrix(values):
    """3x3 tensor from Fusion's {xx, yy, zz, xy, yz, xz} (already in URDF sign convention)."""
    return np.array([
        [values["xx"], values["xy"], values["xz"]],
        [values["xy"], values["yy"], values["yz"]],
        [values["xz"], values["yz"], values["zz"]],
    ])


def shift_inertia_to_center(inertia_about_origin, mass, center):
    """Parallel axis theorem: tensor about the origin -> tensor about the center of mass."""
    c = np.asarray(center, dtype=float)
    return inertia_about_origin - mass * (np.dot(c, c) * np.eye(3) - np.outer(c, c))


def check_inertia(name, inertia):
    """A physical inertia is positive definite and satisfies the triangle inequality."""
    eig = np.linalg.eigvalsh(inertia)
    if eig.min() <= 0 or eig.max() > eig.sum() - eig.max() + 1e-12:
        raise BuildError("link {} has a non-physical inertia (eigenvalues {})".format(name, eig))


class UnionFind:
    def __init__(self, items):
        self.parent = {item: item for item in items}

    def find(self, item):
        while self.parent[item] != item:
            self.parent[item] = self.parent[self.parent[item]]
            item = self.parent[item]
        return item

    def union(self, a, b):
        self.parent[self.find(a)] = self.find(b)


# --------------------------------------------------------------------------
# Model
# --------------------------------------------------------------------------

@dataclass
class Link:
    name: str
    color: list
    parts: list = field(default_factory=list)
    origin_fusion: np.ndarray = None   # frame origin in Fusion coordinates (cm)
    mass: float = 0.0                   # kg
    center_of_mass: np.ndarray = None   # m, in the link frame
    inertia: np.ndarray = None          # kg*m^2 about the center of mass, link axes
    triangles: np.ndarray = None        # m, in the link frame
    collision: dict = None


@dataclass
class Joint:
    name: str
    kind: str          # "revolute" or "fixed"
    parent: str
    child: str
    origin: np.ndarray  # m, child frame origin in the parent frame
    axis: np.ndarray = None
    limits: tuple = None
    source: str = ""    # Fusion joint name / servo, written as a comment
    tilt_deg: float = 0.0


@dataclass
class Model:
    name: str
    document: str
    links: list
    joints: list
    warnings: list


def build_model(export, config, export_dir):
    """Apply robot_config.yaml to the Fusion export. Pure function: writes nothing."""
    warnings = []

    if export.get("format_version") != SUPPORTED_EXPORT_FORMAT:
        raise BuildError("export.json has format_version {}, this script reads {}. Update both "
                         "MiniQbotExport and this script together.".format(
                             export.get("format_version"), SUPPORTED_EXPORT_FORMAT))
    if config.get("format_version") != SUPPORTED_CONFIG_FORMAT:
        raise BuildError("robot_config.yaml has an unsupported format_version")
    prefix = config.get("source_document_prefix", "")
    if not export["document"].startswith(prefix):
        raise BuildError("the export comes from '{}' but robot_config.yaml expects '{}'. Open the "
                         "right design in Fusion and run MiniQbotExport again.".format(
                             export["document"], prefix))

    occurrences = {o["name"]: o for o in export["occurrences"]}
    joints = {j["name"]: j for j in export["joints"]}

    def require_part(name, where):
        if name not in occurrences:
            raise BuildError("{} names the part '{}', which is not in the CAD. Was it renamed?".format(
                where, name))

    def require_joint(name, where):
        if name not in joints:
            raise BuildError("{} names the joint '{}', which is not in the CAD. Was it renamed?".format(
                where, name))

    # ---- which parts are used -------------------------------------------
    exclude = set(config.get("exclude_parts") or [])
    hidden = config.get("hidden_parts") or {}
    not_drawn = set(config.get("not_drawn_parts") or [])
    for name in exclude:
        require_part(name, "exclude_parts")
    for name in hidden:
        require_part(name, "hidden_parts")
    for name in not_drawn:
        require_part(name, "not_drawn_parts")

    parts = {}  # part name -> {"visual": bool}
    for name, occ in occurrences.items():
        if name in exclude:
            continue
        if occ["visible"]:
            parts[name] = {"visual": name not in not_drawn}
        elif name in hidden:
            parts[name] = {"visual": bool(hidden[name].get("visual", True))}
        else:
            raise BuildError("the part '{}' is hidden in the CAD. Add it to exclude_parts (not on "
                             "the robot) or to hidden_parts (on the robot).".format(name))

    # ---- links and their anchors -----------------------------------------
    leg_colors = config["leg_colors"]
    link_specs = [(config["body"]["link"], config["body"]["anchor"], config["body"]["color"])]
    kinematic = []  # (joint config name, parent link, child link, fusion joint, servo, segment)
    for leg, segments in config["legs"].items():
        parent = config["body"]["link"]
        for segment in SEGMENTS:
            spec = segments[segment]
            child = "{}_{}".format(leg, SEGMENT_LINK[segment])
            link_specs.append((child, spec["anchor"], leg_colors[SEGMENT_COLOR[segment]]))
            kinematic.append(("{}_{}".format(leg, SEGMENT_JOINT[segment]), parent, child,
                              spec["joint"], spec.get("servo", ""), segment))
            parent = child

    for link_name, anchor, _ in link_specs:
        require_part(anchor, "link " + link_name)
        if anchor not in parts:
            raise BuildError("the anchor '{}' of link {} is excluded or hidden".format(anchor, link_name))

    # ---- group parts with rigid connections ------------------------------
    merge = set(config.get("merge_joints") or [])
    drop = set(config.get("drop_joints") or [])
    kinematic_names = {k[3] for k in kinematic}
    for name in merge:
        require_joint(name, "merge_joints")
    for name in drop:
        require_joint(name, "drop_joints")
    for name in kinematic_names:
        require_joint(name, "legs")

    groups = UnionFind(parts)
    for joint in export["joints"]:
        if joint.get("suppressed") or joint["name"] in drop or joint["name"] in kinematic_names:
            continue
        ends = [joint.get("occurrence_one"), joint.get("occurrence_two")]
        if any(end is None or end in exclude for end in ends):
            continue
        if joint["motion"].get("type") == "rigid" or joint["name"] in merge:
            groups.union(ends[0], ends[1])
        else:
            raise BuildError("the joint '{}' ({}) moves in the CAD but robot_config.yaml does not "
                             "say what it is. Add it to legs, merge_joints or drop_joints.".format(
                                 joint["name"], joint["motion"].get("type")))
    for group in export.get("rigid_groups") or []:
        members = [m for m in group["occurrences"] if m in parts]
        for a, b in zip(members, members[1:]):
            groups.union(a, b)
    anchors = {name: anchor for name, anchor, _ in link_specs}
    for name, spec in hidden.items():
        if name in parts and spec.get("link"):
            if spec["link"] not in anchors:
                raise BuildError("hidden part '{}' names an unknown link {}".format(name, spec["link"]))
            groups.union(name, anchors[spec["link"]])

    group_to_link = {}
    for link_name, anchor, _ in link_specs:
        root = groups.find(anchor)
        if root in group_to_link:
            raise BuildError("links {} and {} are glued together by rigid joints in the CAD. Check "
                             "merge_joints and the anchors.".format(group_to_link[root], link_name))
        group_to_link[root] = link_name

    links = {name: Link(name=name, color=color) for name, _, color in link_specs}
    orphans = []
    for name in parts:
        link_name = group_to_link.get(groups.find(name))
        if link_name is None:
            orphans.append(name)
        else:
            links[link_name].parts.append(name)
    if orphans:
        raise BuildError("these parts are not connected to any link: {}. Add a rigid joint in the "
                         "CAD, or list them in exclude_parts.".format(", ".join(sorted(orphans))))

    # ---- kinematic joints: check they connect the expected links ---------
    part_link = {p: l.name for l in links.values() for p in l.parts}
    for _, parent, child, fusion_name, _, _ in kinematic:
        joint = joints[fusion_name]
        ends = {part_link.get(joint["occurrence_one"]), part_link.get(joint["occurrence_two"])}
        if ends != {parent, child}:
            raise BuildError("the joint '{}' should connect {} and {}, but connects {}.".format(
                fusion_name, parent, child, " and ".join(sorted(str(e) for e in ends))))
        if joint["motion"].get("type") != "revolute":
            raise BuildError("the joint '{}' must be revolute".format(fusion_name))
        if joint.get("origin_cm") is None or joint["motion"].get("axis") is None:
            raise BuildError("the joint '{}' has no origin or axis in the export".format(fusion_name))
        gap = np.linalg.norm(np.subtract(joint["origin_cm"], joint.get("origin_two_cm") or joint["origin_cm"]))
        if gap > 0.1:
            warnings.append("joint '{}': its two sides are {:.2f} cm apart in the CAD".format(fusion_name, gap))

    # ---- frames ------------------------------------------------------------
    rotation = axis_mapping(config["fusion_to_ros"])
    body_name = config["body"]["link"]
    hip_origins = [joints[k[3]]["origin_cm"] for k in kinematic if k[5] == "hip"]
    links[body_name].origin_fusion = np.mean(hip_origins, axis=0)
    for _, _, child, fusion_name, _, _ in kinematic:
        links[child].origin_fusion = np.asarray(joints[fusion_name]["origin_cm"], dtype=float)

    def to_link_frame(link, points_cm):
        return (np.asarray(points_cm) - link.origin_fusion) @ rotation.T * CM_TO_M

    # ---- mass properties and meshes -------------------------------------
    overrides = config.get("mass_overrides_kg") or {}
    for link in links.values():
        mass, moment, inertia_origin = 0.0, np.zeros(3), np.zeros((3, 3))
        meshes = []
        for part_name in link.parts:
            occ = occurrences[part_name]
            part_mass = sum(b.get("mass_kg", 0.0) for b in occ["bodies"])
            scale = 1.0
            for pattern, real_mass in overrides.items():
                if fnmatch.fnmatchcase(part_name, pattern) and part_mass > 0:
                    scale = real_mass / part_mass
            for body in occ["bodies"]:
                if "mass_kg" not in body or "inertia_about_origin_kgcm2" not in body:
                    raise BuildError("part '{}' body '{}' has no mass properties in the export".format(
                        part_name, body["name"]))
                m = body["mass_kg"] * scale
                mass += m
                moment += m * np.asarray(body["center_of_mass_cm"])
                inertia_origin += scale * inertia_matrix(body["inertia_about_origin_kgcm2"])
                if parts[part_name]["visual"]:
                    if "mesh" not in body:
                        raise BuildError("part '{}' body '{}' has no mesh in the export".format(
                            part_name, body["name"]))
                    meshes.append(read_stl(os.path.join(export_dir, body["mesh"])))
        if mass <= 0:
            raise BuildError("link {} has no mass".format(link.name))
        center_cm = moment / mass
        inertia_center = shift_inertia_to_center(inertia_origin, mass, center_cm)
        link.mass = mass
        link.center_of_mass = to_link_frame(link, center_cm)
        link.inertia = rotation @ inertia_center @ rotation.T * KGCM2_TO_KGM2
        check_inertia(link.name, link.inertia)
        triangles = np.concatenate(meshes) if meshes else np.zeros((0, 3, 3))
        link.triangles = to_link_frame(link, triangles.reshape(-1, 3)).reshape(-1, 3, 3)

    # ---- joints ------------------------------------------------------------
    snap = float(config.get("axis_snap_tolerance_deg", 0.0))
    normalize = bool(config.get("normalize_axis_sign", False))
    limits = config["joint_limits_rad"]
    model_joints = [Joint(name="base_to_body", kind="fixed", parent="base_link", child=body_name,
                          origin=np.zeros(3), source="base_link is a plain frame at the body")]
    for name, parent, child, fusion_name, servo, segment in kinematic:
        joint = joints[fusion_name]
        axis, tilt = clean_axis(rotation @ np.asarray(joint["motion"]["axis"]), snap, normalize)
        if tilt > snap:
            warnings.append("joint {}: axis is {:.1f} deg off the main axis, kept tilted".format(name, tilt))
        origin = (links[child].origin_fusion - links[parent].origin_fusion) @ rotation.T * CM_TO_M
        model_joints.append(Joint(name=name, kind="revolute", parent=parent, child=child,
                                  origin=origin, axis=axis, limits=tuple(limits[segment]),
                                  source="CAD joint '{}', servo {}".format(fusion_name, servo),
                                  tilt_deg=tilt))

    # ---- collisions and feet --------------------------------------------
    radius = float(config["collision"]["foot_sphere_radius_m"])
    body = links[body_name]
    low, high = body.triangles.reshape(-1, 3).min(0), body.triangles.reshape(-1, 3).max(0)
    body.collision = {"box": high - low, "center": (high + low) / 2}
    foot_links = []
    for leg in config["legs"]:
        shin = links["{}_shin_link".format(leg)]
        vertices = shin.triangles.reshape(-1, 3)
        if len(vertices) == 0:
            raise BuildError("{} has no visual mesh to find the foot tip".format(shin.name))
        tip = vertices[np.argmax(np.linalg.norm(vertices, axis=1))]
        center = tip - radius * tip / np.linalg.norm(tip)
        shin.collision = {"sphere": radius, "center": center}
        foot = Link(name="{}_foot".format(leg), color=None)
        foot_links.append(foot)
        model_joints.append(Joint(name="{}_foot_joint".format(leg), kind="fixed", parent=shin.name,
                                  child=foot.name, origin=center,
                                  source="center of the foot contact sphere"))

    ordered = [Link(name="base_link", color=None)] + list(links.values()) + foot_links
    return Model(name=config["robot_name"], document=export["document"], links=ordered,
                 joints=model_joints, warnings=warnings)


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------

def fmt(values):
    return " ".join("{:.6g}".format(0.0 if abs(v) < 1e-12 else float(v)) for v in np.ravel(values))


def render_urdf(model, config, package):
    total = sum(l.mass for l in model.links)
    lines = [
        '<?xml version="1.0"?>',
        "<!--",
        "  GENERATED FILE, DO NOT EDIT BY HAND.",
        "  Source: Fusion design '{}', via tools/fusion_to_urdf (see its README).".format(model.document),
        "  Regenerate: python3 tools/fusion_to_urdf/build_urdf.py",
        "",
        "  All link frames are parallel to base_link. Joint angle 0 = CAD pose.",
        "  Total mass: {:.3f} kg".format(total),
        "-->",
        '<robot name="{}">'.format(model.name),
    ]
    for link in model.links:
        lines.append('  <link name="{}">'.format(link.name))
        if link.mass > 0:
            i = link.inertia
            lines += [
                "    <inertial>",
                '      <origin xyz="{}" rpy="0 0 0"/>'.format(fmt(link.center_of_mass)),
                '      <mass value="{}"/>'.format(fmt([link.mass])),
                '      <inertia ixx="{}" ixy="{}" ixz="{}" iyy="{}" iyz="{}" izz="{}"/>'.format(
                    *(fmt([v]) for v in (i[0, 0], i[0, 1], i[0, 2], i[1, 1], i[1, 2], i[2, 2]))),
                "    </inertial>",
            ]
        if link.triangles is not None and len(link.triangles):
            lines += [
                "    <visual>",
                '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                '      <geometry><mesh filename="package://{}/meshes/{}.stl"/></geometry>'.format(
                    package, link.name),
                '      <material name="{}_color"><color rgba="{}"/></material>'.format(
                    link.name, fmt(link.color)),
                "    </visual>",
            ]
        if link.collision:
            if "box" in link.collision:
                shape = '<box size="{}"/>'.format(fmt(link.collision["box"]))
            else:
                shape = '<sphere radius="{}"/>'.format(fmt([link.collision["sphere"]]))
            lines += [
                "    <collision>",
                '      <origin xyz="{}" rpy="0 0 0"/>'.format(fmt(link.collision["center"])),
                "      <geometry>{}</geometry>".format(shape),
                "    </collision>",
            ]
        lines.append("  </link>")
    lines.append("")
    for joint in model.joints:
        lines.append("  <!-- {} -->".format(joint.source))
        lines.append('  <joint name="{}" type="{}">'.format(joint.name, joint.kind))
        lines.append('    <parent link="{}"/>'.format(joint.parent))
        lines.append('    <child link="{}"/>'.format(joint.child))
        lines.append('    <origin xyz="{}" rpy="0 0 0"/>'.format(fmt(joint.origin)))
        if joint.kind == "revolute":
            lines.append('    <axis xyz="{}"/>'.format(fmt(joint.axis)))
            lines.append('    <limit lower="{}" upper="{}" effort="{}" velocity="{}"/>'.format(
                *(fmt([v]) for v in joint.limits + (config["joint_effort_nm"], config["joint_velocity_rad_s"]))))
            lines.append('    <dynamics damping="{}" friction="{}"/>'.format(
                fmt([config["joint_damping"]]), fmt([config["joint_friction"]])))
        lines.append("  </joint>")
    lines.append("</robot>")
    return "\n".join(lines) + "\n"


def write_package_files(model, config, output_dir):
    package = os.path.basename(os.path.normpath(output_dir))
    urdf_dir = os.path.join(output_dir, "urdf")
    mesh_dir = os.path.join(output_dir, "meshes")
    os.makedirs(urdf_dir, exist_ok=True)
    os.makedirs(mesh_dir, exist_ok=True)
    for old in os.listdir(mesh_dir):
        if old.endswith(".stl"):
            os.remove(os.path.join(mesh_dir, old))
    for link in model.links:
        if link.triangles is not None and len(link.triangles):
            write_stl(os.path.join(mesh_dir, link.name + ".stl"), link.triangles,
                      "{} {} (m)".format(model.name, link.name))
    with open(os.path.join(urdf_dir, model.name + ".urdf"), "w", encoding="utf-8") as out:
        out.write(render_urdf(model, config, package))


def print_summary(model):
    print("Design: {}".format(model.document))
    print("{:<16} {:>6} {:>9} {:>10}".format("link", "parts", "mass [g]", "triangles"))
    for link in model.links:
        if link.mass > 0:
            print("{:<16} {:>6} {:>9.1f} {:>10}".format(
                link.name, len(link.parts), link.mass * 1000, len(link.triangles)))
    print("{:<16} {:>6} {:>9.1f}".format("total", sum(len(l.parts) for l in model.links),
                                         sum(l.mass for l in model.links) * 1000))
    for warning in model.warnings:
        print("WARNING: " + warning)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--export", default=DEFAULT_EXPORT, help="folder written by MiniQbotExport")
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="robot_config.yaml")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="miniqbot_description package folder")
    args = parser.parse_args(argv)
    try:
        with open(os.path.join(args.export, "export.json"), encoding="utf-8") as source:
            export = json.load(source)
        with open(args.config, encoding="utf-8") as source:
            config = yaml.safe_load(source)
        model = build_model(export, config, args.export)
        write_package_files(model, config, args.output)
    except (BuildError, FileNotFoundError) as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 1
    print_summary(model)
    print("Written to {}".format(args.output))
    return 0


if __name__ == "__main__":
    sys.exit(main())
