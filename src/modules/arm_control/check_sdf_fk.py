#!/usr/bin/env python3
"""Compare PX4's configured FK with the SO101 SDF frame chain (not dynamics)."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import numpy as np

root = Path(__file__).resolve().parents[3]
header = (root / "src/modules/arm_control/arm_control.hpp").read_text()
source = (root / "src/modules/arm_control/arm_control.cpp").read_text()
model = ET.parse(root / "Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/so101/so101.sdf").getroot().find("model")

def numbers(text):
    symbols = {"PI": np.pi, "HALF_PI": np.pi / 2, "-PI": -np.pi, "-HALF_PI": -np.pi / 2}
    return [symbols[t.strip()] if t.strip() in symbols else float(t.strip().rstrip("f")) for t in text.split(",")]

def transform(v):
    x, y, z, r, p, w = v
    cr, sr, cp, sp, cw, sw = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(w), np.sin(w)
    rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    rz = np.array([[cw,-sw,0],[sw,cw,0],[0,0,1]])
    t = np.eye(4); t[:3,:3] = rz @ ry @ rx; t[:3,3] = [x,y,z]
    return t

body = header.split("JointOrigin _joint_origins[ARM_DOF]")[1].split("};")[0]
origins = [transform(numbers(v)) for v in re.findall(r"\{([^{}]+)\}", body)]
tool = transform(numbers(re.search(r"_tool_tf = makeRPYTransform\(([^)]+)\)", source)[1]))
names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
sdf_origins = []
parent = "base_link"
for name in names:
    j = model.find(f"./joint[@name='{name}']")
    assert j.findtext("parent") == parent
    assert j.find("pose").get("relative_to") == parent
    assert np.allclose(np.fromstring(j.findtext("axis/xyz"), sep=" "), [0,0,1])
    sdf_origins.append(transform(np.fromstring(j.findtext("pose"), sep=" ")))
    parent = j.findtext("child")
    link_pose = model.find(f"./link[@name='{parent}']/pose")
    assert link_pose.get("relative_to") == name
    assert np.allclose(np.fromstring(link_pose.text, sep=" "), 0)
frame = model.find("./frame[@name='gripper_frame_joint']")
assert frame.get("attached_to") == parent
sdf_tool = transform(np.fromstring(frame.findtext("pose"), sep=" "))
tip = model.find("./frame[@name='gripper_frame_link']")
assert tip.get("attached_to") == "gripper_frame_joint"
assert np.allclose(np.fromstring(tip.findtext("pose"), sep=" "), 0)

def fk(chain, tip, q):
    t = np.eye(4)
    for origin, angle in zip(chain, q):
        t = t @ origin @ transform([0,0,0,0,0,angle])
    return t @ tip

rng = np.random.default_rng(101)
samples = [np.zeros(5), np.array([-.07,-1.175,1.45,1.3,-1.57])]
samples += list(rng.uniform(-1,1,(100,5)))
max_pos = max_rot = 0.
for q in samples:
    a, b = fk(origins, tool, q), fk(sdf_origins, sdf_tool, q)
    max_pos = max(max_pos, np.linalg.norm(a[:3,3] - b[:3,3]))
    max_rot = max(max_rot, np.linalg.norm(a[:3,:3] - b[:3,:3]))
print(f"{len(samples)} poses: max position difference {max_pos:.9f} m; rotation matrix difference {max_rot:.9f}")
assert max_pos < 1e-5 and max_rot < 1e-4
print("PASS: configured frame chains agree. This does not calibrate physical finger center or prove tracking accuracy.")
