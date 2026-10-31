#!/usr/bin/env python3
"""g1_models.py - the models of the humanoid (paper, Sec. VI) from the URDF of the Unitree G1 with 29 joints
that the manufacturer publishes (unitreerobotics/unitree_ros, robots/g1_description/g1_29dof.urdf,
BSD-3-Clause).

  python3 humanoid/tools/g1_models.py <g1_29dof.urdf> humanoid/models

writes into the output directory:
  g1_fixed.urdf      the robot with the pelvis fixed to the world (a massless root link "world" and a
                     fixed joint to the pelvis), visual and collision geometry removed: the model of the
                     control law (RBDL) and, converted with tools/urdf2mjcf.py, of the simulation (MuJoCo);
  g1_left_arm.urdf,  the kinematic chain of one hand for its IK model: world -> pelvis -> the three waist
  g1_right_arm.urdf  joints as fixed joints at 0 (the waist is held at 0 in the tests) -> torso -> the
                     seven joints of the arm -> the palm (the tool point: the child link of
                     <side>_hand_palm_joint, <side>_rubber_hand);
  g1_joints.txt      the 29 revolute joints in the order of the URDF, with their limits and effort.
Every number is copied unchanged. Standard library only.
"""
import os
import sys
import xml.etree.ElementTree as ET

WAIST = ('waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint')


def strip_geometry(root):
    for link in root.findall('link'):
        for tag in ('visual', 'collision'):
            for e in link.findall(tag):
                link.remove(e)
    for e in root.findall('mujoco'):
        root.remove(e)


def add_world(root, base='pelvis'):
    world = ET.Element('link', {'name': 'world'})
    j = ET.Element('joint', {'name': 'world_to_%s' % base, 'type': 'fixed'})
    ET.SubElement(j, 'parent', {'link': 'world'})
    ET.SubElement(j, 'child', {'link': base})
    ET.SubElement(j, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
    # after the materials, before the first link
    idx = next(i for i, e in enumerate(list(root)) if e.tag == 'link')
    root.insert(idx, world)
    root.insert(idx + 1, j)


def write(tree, path):
    ET.indent(tree, space='  ')
    tree.write(path, encoding='utf-8', xml_declaration=True)


def chain(root, tip):
    """the joints from the root link to the link tip, root first"""
    by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
    out = []
    link = tip
    while link in by_child:
        j = by_child[link]
        out.append(j)
        link = j.find('parent').get('link')
    return list(reversed(out))


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    tree = ET.parse(src)
    root = tree.getroot()
    for j in root.findall('joint'):
        if j.get('type') == 'floating':
            sys.exit('g1_models: the URDF has a floating joint; use the file with it commented out')
    strip_geometry(root)
    add_world(root)
    root.set('name', 'g1_29dof_fixed_pelvis')
    write(tree, os.path.join(out, 'g1_fixed.urdf'))

    rev = [j for j in root.findall('joint') if j.get('type') == 'revolute']
    with open(os.path.join(out, 'g1_joints.txt'), 'w') as f:
        f.write('# %d revolute joints of %s (pelvis fixed): index name lower upper effort velocity\n'
                % (len(rev), os.path.basename(src)))
        for i, j in enumerate(rev):
            lim = j.find('limit')
            f.write('%2d %-28s %9s %9s %6s %6s\n' % (i, j.get('name'), lim.get('lower'), lim.get('upper'),
                                                   lim.get('effort'), lim.get('velocity')))

    links = {l.get('name'): l for l in root.findall('link')}
    for side in ('left', 'right'):
        # the tool point: the child link of <side>_hand_palm_joint (left_rubber_hand in g1_29dof.urdf)
        pj = [j for j in root.findall('joint') if j.get('name') == '%s_hand_palm_joint' % side]
        if not pj:
            sys.exit('g1_models: no joint %s_hand_palm_joint' % side)
        tip = pj[0].find('child').get('link')
        joints = chain(root, tip)
        keep_links = {'world'} | {j.find('child').get('link') for j in joints}
        arm = ET.Element('robot', {'name': 'g1_%s_arm' % side})
        for name in ['world'] + [j.find('child').get('link') for j in joints]:
            arm.append(links[name])
        nrev = 0
        for j in joints:
            jj = ET.fromstring(ET.tostring(j))
            if jj.get('name') in WAIST:
                jj.set('type', 'fixed')         # held at 0: the joint origin is the transform
                for tag in ('axis', 'limit', 'dynamics'):
                    for e in jj.findall(tag):
                        jj.remove(e)
            elif jj.get('type') == 'revolute':
                nrev += 1
            arm.append(jj)
        if nrev != 7:
            sys.exit('g1_models: %d revolute joints in the chain of the %s hand, 7 expected' % (nrev, side))
        missing = keep_links - set(links)
        if missing:
            sys.exit('g1_models: links missing: %s' % missing)
        write(ET.ElementTree(arm), os.path.join(out, 'g1_%s_arm.urdf' % side))
    print('g1_models: %s -> %s: g1_fixed.urdf (%d revolute joints), g1_left_arm.urdf, g1_right_arm.urdf (7 each, '
          'tool point <side>_rubber_hand), g1_joints.txt' % (src, out, len(rev)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
