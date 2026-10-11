#!/usr/bin/env python3
"""Convert the CAD export urdf/ava_robotic_arm.urdf into urdf/ava.kinematics.xacro.

The CAD tool re-exports ava_robotic_arm.urdf wholesale on every design change, so the
links and joints are never hand-edited. This script adapts that raw export to the rest
of the stack, which the exporter knows nothing about:

  * mesh paths           relative "meshes/x.stl" -> "package://ava_description/meshes/x.stl"
  * collision geometry   stripped from the cosmetic links (bearings, spacers, servo
                         bodies, horns, covers) so MoveIt only checks the structural
                         bodies in STRUCTURAL_LINKS. They stay as visuals. This is why
                         move_group logs 38 "visual geometry but no collision geometry"
                         warnings on startup - expected, see the package README.
  * joint limits         the six actuated joints get ${..._lower}/${..._upper} and
                         ${..._effort} from ava.control.xacro instead of baked numbers,
                         so limits remain reviewable in one place.
  * materials            the exporter emits none; visuals are tagged charcoal (servos,
                         bearings, hardware) or light_grey (printed structure).

Everything the exporter does NOT provide - base_footprint, end_effector_link, the wrist
camera frames, payload, ros2_control, Gazebo - lives in ava.urdf.xacro and is unaffected.

Usage:  scripts/generate_kinematics_xacro.py        (no ROS needed)
"""
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
SRC = PKG / "urdf" / "ava_robotic_arm.urdf"
OUT = PKG / "urdf" / "ava.kinematics.xacro"

# Load-bearing printed parts. Only these keep <collision>; see module docstring.
STRUCTURAL_LINKS = {
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_link",
    "gripper_base_link",
    "gripper_jaw_link",
}

# Actuated joints, in bus order (motor IDs 1-6 in ava.control.xacro).
ACTUATED = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Visual material by link-name prefix; anything unmatched is printed structure.
CHARCOAL_RE = re.compile(r"^(mg996R|sg90R|sg90r|horn_sg90|bearing_|spacer_|axis_)")


def material_for(link_name):
    return "charcoal" if CHARCOAL_RE.match(link_name) else "light_grey"


def indent(elem, level=0):
    pad = "\n" + "  " * level
    if len(elem):
        if not (elem.text or "").strip():
            elem.text = pad + "  "
        for child in elem:
            indent(child, level + 1)
        if not (elem[-1].tail or "").strip():
            elem[-1].tail = pad
    if level and not (elem.tail or "").strip():
        elem.tail = pad


def main():
    if not SRC.exists():
        sys.exit(f"ERROR: {SRC} not found")
    root = ET.parse(SRC).getroot()

    link_names = {l.get("name") for l in root.findall("link")}
    missing = STRUCTURAL_LINKS - link_names
    if missing:
        sys.exit(
            "ERROR: the export no longer contains these STRUCTURAL_LINKS: "
            + ", ".join(sorted(missing))
            + "\n       The CAD part names changed - update STRUCTURAL_LINKS in this script."
        )

    out = ET.Element("robot", {"xmlns:xacro": "http://www.ros.org/wiki/xacro"})

    stripped = 0
    for link in root.findall("link"):
        name = link.get("name")
        if name not in STRUCTURAL_LINKS:
            for col in link.findall("collision"):
                link.remove(col)
                stripped += 1
        for vis in link.findall("visual"):
            # Drop the exporter's per-visual name= and give the visual a material.
            vis.attrib.pop("name", None)
            if vis.find("material") is None:
                ET.SubElement(vis, "material", {"name": material_for(name)})
        for col in link.findall("collision"):
            col.attrib.pop("name", None)
        out.append(link)

    found = set()
    for joint in root.findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if limit is not None and name in ACTUATED:
            found.add(name)
            # velocity="1e6" (unreachable) disables JointSaturationLimiter's clipping;
            # the real ceiling comes from max_velocity in the <ros2_control> block.
            limit.set("lower", "${%s_lower}" % name)
            limit.set("upper", "${%s_upper}" % name)
            limit.set("effort", "${%s_effort}" % name)
            limit.set("velocity", "1e6")
        out.append(joint)

    if found != set(ACTUATED):
        sys.exit(
            "ERROR: expected actuated joints %s, export has %s"
            % (sorted(ACTUATED), sorted(found))
        )

    # Mesh paths must resolve through the ament index, not the cwd.
    for mesh in out.iter("mesh"):
        fn = mesh.get("filename")
        if not fn.startswith("package://"):
            mesh.set("filename", "package://ava_description/" + fn.lstrip("./"))

    indent(out)
    body = ET.tostring(out, encoding="unicode")
    OUT.write_text(
        '<?xml version="1.0"?>\n'
        "<!-- GENERATED by scripts/generate_kinematics_xacro.py from urdf/ava_robotic_arm.urdf\n"
        "     DO NOT EDIT. Re-export the CAD model, then re-run that script. -->\n"
        + body
        + "\n"
    )
    print(
        f"OK: wrote {OUT.relative_to(PKG)} "
        f"({len(out.findall('link'))} links, {len(out.findall('joint'))} joints, "
        f"collision kept on {len(STRUCTURAL_LINKS)}, {stripped} cosmetic collisions stripped)"
    )


if __name__ == "__main__":
    main()
