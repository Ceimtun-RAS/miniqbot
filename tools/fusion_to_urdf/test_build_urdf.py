"""
Unit tests for build_urdf.py. They use a small synthetic robot (boxes),
so they run without Fusion and without the real export.

    python3 -m unittest discover -s tools/fusion_to_urdf -v
"""

import copy
import os
import tempfile
import unittest

import numpy as np

import build_urdf as bd

# Same axis convention as the real CAD: front = +Z, left = +X, up = +Y.
FUSION_TO_ROS = {"x": "+z", "y": "+x", "z": "+y"}


def box_triangles(center, size):
    """12 outward-facing triangles of an axis-aligned box (cm)."""
    c, h = np.asarray(center, float), np.asarray(size, float) / 2
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h + c
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    triangles = []
    for a, b, cc, d in faces:
        triangles += [corners[[a, b, cc]], corners[[a, cc, d]]]
    return np.array(triangles)


def box_body(name, center, size, mass, mesh_dir, index):
    """A body entry as MiniQbotExport writes it, with exact box inertia about the origin."""
    a, b, c = size
    p = np.asarray(center, float)
    inertia = np.diag([b * b + c * c, a * a + c * c, a * a + b * b]) * mass / 12
    inertia += mass * (np.dot(p, p) * np.eye(3) - np.outer(p, p))
    mesh = "meshes/{:03d}.stl".format(index)
    bd.write_stl(os.path.join(mesh_dir, os.path.basename(mesh)), box_triangles(center, size), "test")
    return {"name": name, "mass_kg": mass, "volume_cm3": a * b * c, "center_of_mass_cm": list(p),
            "inertia_about_origin_kgcm2": {"xx": inertia[0, 0], "yy": inertia[1, 1], "zz": inertia[2, 2],
                                           "xy": inertia[0, 1], "yz": inertia[1, 2], "xz": inertia[0, 2]},
            "mesh": mesh}


def make_robot(folder):
    """Body + one leg 'l' (hip, thigh with a screw part, shin) + one hidden and one loose part."""
    mesh_dir = os.path.join(folder, "meshes")
    os.makedirs(mesh_dir)
    parts = [  # name, center, size, mass, visible
        ("body:1", (0, 0, 0), (10, 2, 10), 0.5, True),
        ("M1:1", (5, 0, 5), (2, 2, 2), 0.04, True),
        ("hip:1", (7, 0, 6), (2, 2, 2), 0.01, True),
        ("thigh:1", (8, 0, 3), (1, 1, 10), 0.02, True),
        ("screw:1", (8.5, 0, 3), (0.2, 0.2, 2), 0.001, True),
        ("shin:1", (8, -5, -2), (1, 10, 1), 0.02, True),
        ("lid:1", (0, 1.5, 0), (10, 1, 10), 0.1, False),
        ("loose:1", (30, 0, 0), (1, 1, 1), 0.001, False),
    ]
    occurrences = []
    for index, (name, center, size, mass, visible) in enumerate(parts):
        occurrences.append({"name": name, "component": name, "light_bulb_on": visible, "visible": visible,
                            "bodies": [box_body("Body1", center, size, mass, mesh_dir, index)]})

    def joint(name, kind, one, two, origin=None, axis=None):
        return {"name": name, "occurrence_one": one, "occurrence_two": two, "suppressed": False,
                "motion": {"type": kind, "axis": axis}, "origin_cm": origin, "origin_two_cm": origin}

    joints = [
        joint("rigid motor", "rigid", "M1:1", "body:1"),
        joint("j_hip", "revolute", "hip:1", "M1:1", [6, 0, 6], [0, 0, -1]),
        joint("j_thigh", "revolute", "thigh:1", "hip:1", [8, 0, 8], [0.999, -0.04, 0]),
        joint("screw drive", "cylindrical", "screw:1", "thigh:1", [8.5, 0, 3], [0, 0, 1]),
        joint("j_knee", "revolute", "shin:1", "thigh:1", [8, 0, -2], [1, 0, 0]),
        joint("loop closer", "revolute", "screw:1", "shin:1", [8.5, 0, -1], [1, 0, 0]),
    ]
    return {"format_version": 1, "document": "Test robot v1", "occurrences": occurrences,
            "joints": joints, "rigid_groups": []}


CONFIG = {
    "format_version": 1,
    "source_document_prefix": "Test robot",
    "robot_name": "testbot",
    "fusion_to_ros": FUSION_TO_ROS,
    "axis_snap_tolerance_deg": 10.0,
    "normalize_axis_sign": True,
    "body": {"link": "body_link", "anchor": "body:1", "color": [0.2, 0.2, 0.2, 1]},
    "legs": {"l": {"hip": {"anchor": "hip:1", "joint": "j_hip", "servo": "M1"},
                   "thigh": {"anchor": "thigh:1", "joint": "j_thigh"},
                   "knee": {"anchor": "shin:1", "joint": "j_knee"}}},
    "leg_colors": {"hip": [0, 0, 1, 1], "thigh": [0, 1, 0, 1], "shin": [1, 0, 0, 1]},
    "merge_joints": ["screw drive"],
    "drop_joints": ["loop closer"],
    "exclude_parts": ["loose:1"],
    "hidden_parts": {"lid:1": {"link": "body_link", "visual": True}},
    "mass_overrides_kg": {"M?:1": 0.05},
    "joint_limits_rad": {"hip": [-0.5, 0.5], "thigh": [-1, 1], "knee": [-1, 1]},
    "joint_effort_nm": 1.5,
    "joint_velocity_rad_s": 6.0,
    "joint_damping": 0.05,
    "joint_friction": 0.02,
    "collision": {"foot_sphere_radius_m": 0.005},
}


class HelperTests(unittest.TestCase):
    def test_axis_mapping_is_the_cad_convention(self):
        rotation = bd.axis_mapping(FUSION_TO_ROS)
        # Fusion +Z (robot front) -> ROS +x; Fusion +X (left) -> +y; Fusion +Y (up) -> +z.
        np.testing.assert_allclose(rotation @ [0, 0, 1], [1, 0, 0])
        np.testing.assert_allclose(rotation @ [1, 0, 0], [0, 1, 0])
        np.testing.assert_allclose(rotation @ [0, 1, 0], [0, 0, 1])

    def test_axis_mapping_rejects_mirror(self):
        with self.assertRaises(bd.BuildError):
            bd.axis_mapping({"x": "+z", "y": "-x", "z": "+y"})

    def test_clean_axis_snaps_small_tilt_and_normalizes_sign(self):
        axis, tilt = bd.clean_axis([-0.997, 0.08, 0], 10.0, True)
        np.testing.assert_allclose(axis, [1, 0, 0])
        self.assertAlmostEqual(tilt, 4.59, places=1)

    def test_clean_axis_keeps_large_tilt(self):
        axis, tilt = bd.clean_axis([0, 0.6, 0.8], 10.0, False)
        np.testing.assert_allclose(axis, [0, 0.6, 0.8])
        self.assertGreater(tilt, 10.0)

    def test_parallel_axis_round_trip(self):
        center = np.array([1.0, -2.0, 3.0])
        inertia_center = np.diag([1.0, 2.0, 2.5])
        about_origin = inertia_center + 2.0 * (center @ center * np.eye(3) - np.outer(center, center))
        np.testing.assert_allclose(bd.shift_inertia_to_center(about_origin, 2.0, center), inertia_center)

    def test_check_inertia_rejects_non_physical(self):
        with self.assertRaises(bd.BuildError):
            bd.check_inertia("bad", np.diag([1.0, 1.0, 3.0]))

    def test_stl_round_trip(self):
        triangles = box_triangles((1, 2, 3), (2, 4, 6))
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "box.stl")
            bd.write_stl(path, triangles, "box")
            np.testing.assert_allclose(bd.read_stl(path), triangles, atol=1e-6)


class BuildModelTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.export = make_robot(self.folder.name)
        self.config = copy.deepcopy(CONFIG)

    def tearDown(self):
        self.folder.cleanup()

    def build(self):
        return bd.build_model(self.export, self.config, self.folder.name)

    def links(self, model):
        return {link.name: link for link in model.links}

    def joints(self, model):
        return {joint.name: joint for joint in model.joints}

    def test_parts_are_grouped_into_links(self):
        links = self.links(self.build())
        self.assertEqual(sorted(links["body_link"].parts), ["M1:1", "body:1", "lid:1"])
        self.assertEqual(sorted(links["l_thigh_link"].parts), ["screw:1", "thigh:1"])
        self.assertEqual(links["l_shin_link"].parts, ["shin:1"])

    def test_mass_override_and_hidden_part_mass(self):
        links = self.links(self.build())
        self.assertAlmostEqual(links["body_link"].mass, 0.5 + 0.05 + 0.1)

    def test_not_drawn_part_keeps_its_mass(self):
        drawn = self.links(self.build())["l_thigh_link"]
        self.config["not_drawn_parts"] = ["screw:1"]
        hidden = self.links(self.build())["l_thigh_link"]
        self.assertAlmostEqual(hidden.mass, drawn.mass)
        self.assertEqual(len(hidden.triangles), len(drawn.triangles) - 12)

    def test_joint_origins_and_axes_in_ros_frame(self):
        joints = self.joints(self.build())
        # Body frame = hip joint centroid (6, 0, 6) cm. Hip joint at the same point.
        np.testing.assert_allclose(joints["l_hip_joint"].origin, [0, 0, 0], atol=1e-12)
        # Thigh joint 2 cm further front (+Z) and left (+X) in Fusion -> +x and +y in ROS.
        np.testing.assert_allclose(joints["l_thigh_joint"].origin, [0.02, 0.02, 0], atol=1e-12)
        # Knee 10 cm behind the thigh joint.
        np.testing.assert_allclose(joints["l_knee_joint"].origin, [-0.10, 0, 0], atol=1e-12)
        np.testing.assert_allclose(joints["l_hip_joint"].axis, [1, 0, 0])    # Fusion -Z, flipped
        np.testing.assert_allclose(joints["l_thigh_joint"].axis, [0, 1, 0])  # snapped
        self.assertEqual(joints["l_hip_joint"].limits, (-0.5, 0.5))

    def test_link_inertia_is_about_its_center_of_mass(self):
        shin = self.links(self.build())["l_shin_link"]
        # Shin box 1 x 10 x 1 cm (Fusion), 20 g: long side along Fusion Y = ROS z.
        expected = np.diag([1 + 100, 1 + 100, 1 + 1]) * 0.02 / 12 * 1e-4
        np.testing.assert_allclose(shin.inertia, expected, atol=1e-12)
        # Its center is 5 cm below the knee.
        np.testing.assert_allclose(shin.center_of_mass, [0, 0, -0.05], atol=1e-12)

    def test_foot_is_at_the_far_end_of_the_shin(self):
        model = self.build()
        foot = self.joints(model)["l_foot_joint"]
        self.assertAlmostEqual(foot.origin[2], -0.10 + 0.005 * 0.99, delta=0.002)
        self.assertIn("sphere", self.links(model)["l_shin_link"].collision)

    def test_meshes_are_in_the_link_frame_in_meters(self):
        shin = self.links(self.build())["l_shin_link"]
        vertices = shin.triangles.reshape(-1, 3)
        np.testing.assert_allclose(vertices.max(0), [0.005, 0.005, 0.0], atol=1e-9)
        np.testing.assert_allclose(vertices.min(0), [-0.005, -0.005, -0.10], atol=1e-9)

    def test_rendered_urdf_contains_every_link_and_joint(self):
        model = self.build()
        text = bd.render_urdf(model, self.config, "testbot_description")
        for link in model.links:
            self.assertIn('<link name="{}">'.format(link.name), text)
        for joint in model.joints:
            self.assertIn('<joint name="{}"'.format(joint.name), text)
        self.assertIn("package://testbot_description/meshes/body_link.stl", text)
        self.assertIn('<limit lower="-0.5" upper="0.5" effort="1.5" velocity="6"/>', text)

    def test_wrong_document_is_rejected(self):
        self.export["document"] = "Some other design v3"
        with self.assertRaisesRegex(bd.BuildError, "expects"):
            self.build()

    def test_hidden_part_not_listed_is_rejected(self):
        del self.config["hidden_parts"]["lid:1"]
        with self.assertRaisesRegex(bd.BuildError, "lid:1"):
            self.build()

    def test_unclassified_moving_joint_is_rejected(self):
        self.config["merge_joints"] = []
        with self.assertRaisesRegex(bd.BuildError, "screw drive"):
            self.build()

    def test_renamed_part_is_reported(self):
        self.config["legs"]["l"]["knee"]["anchor"] = "shin v2:1"
        with self.assertRaisesRegex(bd.BuildError, "shin v2:1"):
            self.build()

    def test_part_without_connection_is_reported(self):
        self.export["joints"] = [j for j in self.export["joints"] if j["name"] != "rigid motor"]
        with self.assertRaisesRegex(bd.BuildError, "M1:1"):
            self.build()

    def test_joint_between_wrong_links_is_reported(self):
        self.config["legs"]["l"]["knee"]["joint"] = "j_thigh"
        self.config["legs"]["l"]["thigh"]["joint"] = "j_knee"
        with self.assertRaisesRegex(bd.BuildError, "should connect"):
            self.build()

    def test_write_package_files(self):
        model = self.build()
        with tempfile.TemporaryDirectory() as output:
            package = os.path.join(output, "testbot_description")
            bd.write_package_files(model, self.config, package)
            self.assertTrue(os.path.isfile(os.path.join(package, "urdf", "testbot.urdf")))
            meshes = sorted(os.listdir(os.path.join(package, "meshes")))
            self.assertEqual(meshes, ["body_link.stl", "l_hip_link.stl", "l_shin_link.stl", "l_thigh_link.stl"])


if __name__ == "__main__":
    unittest.main()
