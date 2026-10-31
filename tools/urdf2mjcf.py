#!/usr/bin/env python3
"""urdf2mjcf.py - convert the Indy7 URDF into the MuJoCo model of the simulation
(indy7/models/indy7_mujoco.xml; paper, Sec. IV-A).

Every number of the URDF is copied into the MJCF unchanged, as text:
  joint origin  -> body pos / euler   (compiler eulerseq="XYZ": the URDF rpy convention,
                                       rotations about the fixed x, y, z axes in this order)
  inertial      -> inertial pos / euler / mass / fullinertia
  joint axis    -> joint axis, limits -> joint range
MuJoCo's own URDF importer gives the same model, but the MJCF it saves rounds every number
to 6 significant digits, which alone makes M(q) differ from RBDL by ~1e-6.

Deliberately not copied (the model of the controller has none of them either):
  - <dynamics damping friction>: MuJoCo's importer would turn them into joint damping and a
    dry-friction torque (frictionloss). Here damping = frictionloss = armature = 0; the
    simulation switches frictionloss on at run time with the breakaway torques of the robot.
  - <visual>, <collision>: geometry plays no part in the simulation or the checks.
  - effort and velocity limits.
The integrator is RK4 (--integrator): MuJoCo's default, semi-implicit Euler, advances the
positions half a step ahead of the velocities, which at the step of 1 ms adds a tracking error
of its own, of the size of the error that the delay of the controller causes (about 50 % at
J2 in the fast test, tools/jt_predict.py); RK4 integrates the arm under the torque held over
the step, as the drives hold it.

Usage:
  python3 tools/urdf2mjcf.py indy7/models/indy7.urdf indy7/models/indy7_mujoco.xml [--timestep 0.001]
                            [--integrator RK4] [--verify]
  python3 tools/urdf2mjcf.py indy7/models/indy7.urdf <scene.xml> --scene --meshdir <dir of the visual meshes>
--verify (needs the mujoco Python package): loads the URDF with MuJoCo's own importer as well
and compares the two models over random states (body poses, mass matrix, bias forces); the
differences must be at the level of double precision.
--scene: the same model plus the visual meshes of the URDF (looked up by file name in
--meshdir), a floor, lights, and a camera, for viewing and figures (tools/sim_view.py). The
meshes only show: no collision (contype = conaffinity = 0), no mass (density 0, and every
link has its own <inertial>), so the dynamics are those of the model above; check it with
--verify or tools/model_check. The real-time simulation keeps the model without geometry.
"""
import argparse
import hashlib
import os
import sys
import xml.etree.ElementTree as ET
from xml.sax.saxutils import quoteattr


def norm(text, default):
    """URDF numbers as given, with the whitespace normalized; nothing is reformatted."""
    return ' '.join((text if text is not None else default).split())


def origin_of(elem):
    o = elem.find('origin') if elem is not None else None
    if o is None:
        return '0 0 0', '0 0 0'
    return norm(o.get('xyz'), '0 0 0'), norm(o.get('rpy'), '0 0 0')


def frame_attrs(xyz, rpy):
    attrs = ' pos="%s"' % xyz
    if any(float(v) != 0.0 for v in rpy.split()):
        attrs += ' euler="%s"' % rpy
    return attrs


def convert(urdf_path, meshes=None):
    """meshes: None, or a dict that receives {mesh name: file name} of the visual meshes"""
    root = ET.parse(urdf_path).getroot()
    links = {l.get('name'): l for l in root.findall('link')}
    joints = root.findall('joint')
    children = {}
    child_links = set()
    for j in joints:
        children.setdefault(j.find('parent').get('link'), []).append(j)
        child_links.add(j.find('child').get('link'))
    roots = [n for n in links if n not in child_links]
    if len(roots) != 1:
        sys.exit('urdf2mjcf: expected one root link, found %s' % roots)

    out = []

    def inertial(link, ind):
        i = link.find('inertial')
        if i is None:
            return
        xyz, rpy = origin_of(i)
        mass = norm(i.find('mass').get('value'), '0')
        t = i.find('inertia')
        full = ' '.join(norm(t.get(k), '0') for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'))
        out.append('%s<inertial%s mass="%s" fullinertia="%s"/>' % (ind, frame_attrs(xyz, rpy), mass, full))

    def visual(link, ind):
        for k, v in enumerate(link.findall('visual')):
            m = v.find('geometry/mesh')
            if m is None:
                continue
            fname = m.get('filename').split('/')[-1]
            mname = '%s_visual%d' % (link.get('name'), k)
            meshes[mname] = (fname, norm(m.get('scale'), '1 1 1'))
            xyz, rpy = origin_of(v)
            out.append('%s<geom class="visual" type="mesh" mesh="%s"%s/>' % (ind, mname, frame_attrs(xyz, rpy)))

    def body(joint, ind):
        name = joint.find('child').get('link')
        link = links[name]
        xyz, rpy = origin_of(joint)
        out.append('%s<body name=%s%s>' % (ind, quoteattr(name), frame_attrs(xyz, rpy)))
        jtype = joint.get('type')
        if jtype in ('revolute', 'continuous', 'prismatic'):
            a = joint.find('axis')
            axis = norm(a.get('xyz') if a is not None else None, '1 0 0')
            attrs = ' name=%s type="%s" axis="%s"' % (quoteattr(joint.get('name')),
                                                     'slide' if jtype == 'prismatic' else 'hinge', axis)
            lim = joint.find('limit')
            if jtype != 'continuous' and lim is not None and lim.get('lower') is not None:
                attrs += ' range="%s %s"' % (norm(lim.get('lower'), '0'), norm(lim.get('upper'), '0'))
            out.append('%s  <joint%s/>' % (ind, attrs))
        elif jtype != 'fixed':
            sys.exit('urdf2mjcf: joint type %s of %s is not supported' % (jtype, joint.get('name')))
        inertial(link, ind + '  ')
        if meshes is not None:
            visual(link, ind + '  ')
        for c in children.get(name, []):
            body(c, ind + '  ')
        out.append('%s</body>' % ind)

    root_link = links[roots[0]]
    if root_link.find('inertial') is not None:
        sys.exit('urdf2mjcf: the root link %s has mass; a fixed base is expected' % roots[0])
    for c in children.get(roots[0], []):
        body(c, '    ')
    return root.get('name', 'robot'), out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('urdf')
    ap.add_argument('mjcf')
    ap.add_argument('--timestep', default='0.001', help='simulator step [s]: one step per control cycle')
    ap.add_argument('--integrator', default='RK4', choices=['RK4', 'Euler', 'implicit', 'implicitfast'],
                    help='MuJoCo integrator (default RK4, see above)')
    ap.add_argument('--verify', action='store_true')
    ap.add_argument('--scene', action='store_true', help='add visual meshes, floor, lights, camera')
    ap.add_argument('--meshdir', help='directory of the visual mesh files (with --scene)')
    a = ap.parse_args()
    if a.scene and not a.meshdir:
        sys.exit('urdf2mjcf: --scene needs --meshdir')

    sha = hashlib.sha256(open(a.urdf, 'rb').read()).hexdigest()
    meshes = {} if a.scene else None
    model, bodies = convert(a.urdf, meshes)
    head = [
        '<mujoco model=%s>' % quoteattr(model),
        '  <!-- generated by tools/urdf2mjcf.py; do not edit, regenerate instead',
        '       source: %s' % os.path.basename(a.urdf),
        '       sha256: %s' % sha,
        '       every number is copied from the URDF; no armature, damping, or friction (switched',
        '       on at run time by the simulation), %s. -->' % (
            'visual meshes only (no collision, no mass), for viewing' if a.scene else 'no geometry'),
        '  <compiler angle="radian" eulerseq="XYZ" autolimits="true"%s/>' % (
            ' meshdir=%s' % quoteattr(a.meshdir) if a.scene else ''),
        '  <option timestep="%s" integrator="%s"/>' % (a.timestep, a.integrator),
        '  <default>',
        '    <joint damping="0" frictionloss="0" armature="0"/>',
    ]
    if a.scene:
        head += [
            '    <default class="visual">',
            '      <geom contype="0" conaffinity="0" density="0" group="1" material="robot"/>',
            '    </default>',
            '  </default>',
            '  <visual>',
            '    <global offwidth="2400" offheight="1800"/>',
            '    <quality shadowsize="4096" offsamples="8"/>',
            '    <headlight ambient="0.48 0.48 0.48" diffuse="0.42 0.42 0.42" specular="0.1 0.1 0.1"/>',
            '  </visual>',
            '  <asset>',
            '    <texture name="sky" type="skybox" builtin="gradient" rgb1="1 1 1" rgb2="1 1 1" width="512" height="512"/>',
            '    <material name="robot" rgba="0.76 0.77 0.79 1" specular="0.3" shininess="0.3"/>',
            '    <texture name="grid" type="2d" builtin="checker" rgb1="0.985 0.985 0.985" rgb2="0.93 0.93 0.94" width="256" height="256"/>',
            '    <material name="floor" texture="grid" texrepeat="5 5" texuniform="true" emission="0.30" specular="0"/>',
        ] + ['    <mesh name="%s" file="%s" scale="%s"/>' % (n, f, sc) for n, (f, sc) in sorted(meshes.items())] + [
            '  </asset>',
            '  <worldbody>',
            '    <light name="key" pos="1.2 -1.0 2.5" dir="-0.4 0.4 -1" diffuse="0.32 0.32 0.32" castshadow="true"/>',
            '    <geom name="floor" type="plane" size="12 12 0.1" material="floor" contype="0" conaffinity="0"/>',
            '    <camera name="figure" pos="1.45 -1.25 0.95" xyaxes="0.65 0.76 0 -0.25 0.21 0.95" fovy="40"/>',
        ]
    else:
        head += ['  </default>', '  <worldbody>']
    text = '\n'.join(head + bodies + ['  </worldbody>', '</mujoco>', ''])
    with open(a.mjcf, 'w') as f:
        f.write(text)
    print('wrote %s (%d bodies, from %s, sha256 %s...)' % (a.mjcf, sum('<body ' in l for l in bodies), a.urdf, sha[:12]))
    if a.verify:
        sys.exit(verify(a.urdf, a.mjcf))


def verify(urdf_path, mjcf_path):
    import tempfile
    import numpy as np
    import mujoco

    ours = mujoco.MjModel.from_xml_path(mjcf_path)
    # MuJoCo's importer on the same URDF, geometry stripped, static bodies kept
    t = ET.parse(urdf_path)
    r = t.getroot()
    for link in r.findall('link'):
        for tag in ('visual', 'collision'):
            for e in link.findall(tag):
                link.remove(e)
    ET.SubElement(ET.SubElement(r, 'mujoco'), 'compiler', {'fusestatic': 'false', 'angle': 'radian'})
    fd, tmp = tempfile.mkstemp(suffix='.urdf')
    os.close(fd)
    t.write(tmp)
    ref = mujoco.MjModel.from_xml_path(tmp)
    os.remove(tmp)

    names = lambda m, obj, n: [mujoco.mj_id2name(m, obj, i) for i in range(n)]
    if names(ours, mujoco.mjtObj.mjOBJ_BODY, ours.nbody) != names(ref, mujoco.mjtObj.mjOBJ_BODY, ref.nbody) or \
       names(ours, mujoco.mjtObj.mjOBJ_JOINT, ours.njnt) != names(ref, mujoco.mjtObj.mjOBJ_JOINT, ref.njnt):
        print('verify: FAIL, bodies or joints differ from the MuJoCo importer')
        return 1
    print('verify: %d bodies, %d joints, nv=%d; total mass %.10g kg (importer %.10g)' % (
        ours.nbody, ours.njnt, ours.nv, ours.body_mass.sum(), ref.body_mass.sum()))
    print('verify: damping %s, frictionloss %s, armature %s (importer: damping %s, frictionloss %s)' % (
        ours.dof_damping.max(), ours.dof_frictionloss.max(), ours.dof_armature.max(),
        ref.dof_damping.max(), ref.dof_frictionloss.max()))
    print('verify: joint ranges equal: %s' % np.array_equal(ours.jnt_range, ref.jnt_range))

    def full_m(m, d):
        # M(q) column by column: mj_mulM has the same signature in every MuJoCo 3
        out, e, col = np.zeros((m.nv, m.nv)), np.zeros(m.nv), np.zeros(m.nv)
        for j in range(m.nv):
            e[j] = 1.0
            mujoco.mj_mulM(m, d, col, e)
            e[j] = 0.0
            out[:, j] = col
        return out

    rng = np.random.default_rng(1)
    d1, d2 = mujoco.MjData(ours), mujoco.MjData(ref)
    worst = dict(xpos=0.0, xmat=0.0, M=0.0, bias=0.0)
    for _ in range(2000):
        q = rng.uniform(ours.jnt_range[:, 0], ours.jnt_range[:, 1])
        qd = rng.uniform(-np.pi, np.pi, ours.nv)
        for m, d in ((ours, d1), (ref, d2)):
            d.qpos[:] = q
            d.qvel[:] = qd
            mujoco.mj_forward(m, d)
        M1, M2 = full_m(ours, d1), full_m(ref, d2)
        worst['xpos'] = max(worst['xpos'], np.abs(d1.xpos - d2.xpos).max())
        worst['xmat'] = max(worst['xmat'], np.abs(d1.xmat - d2.xmat).max())
        worst['M'] = max(worst['M'], np.linalg.norm(M1 - M2) / np.linalg.norm(M2))
        worst['bias'] = max(worst['bias'], np.linalg.norm(d1.qfrc_bias - d2.qfrc_bias) / np.linalg.norm(d2.qfrc_bias))
    print('verify: 2000 random states vs the MuJoCo importer: body position %.2e m, orientation %.2e, '
          'M rel %.2e, bias rel %.2e' % (worst['xpos'], worst['xmat'], worst['M'], worst['bias']))
    ok = worst['xpos'] < 1e-12 and worst['xmat'] < 1e-12 and worst['M'] < 1e-12 and worst['bias'] < 1e-12
    print('verify: %s' % ('PASS (same model as the MuJoCo importer, to double precision)' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    main()
