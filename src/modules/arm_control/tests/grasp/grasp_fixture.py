#!/usr/bin/env python3
"""Generate an isolated dynamic gripper fixture from the current production SDF."""
from pathlib import Path
import copy
import xml.etree.ElementTree as E
import argparse

ROOT = Path(__file__).resolve().parents[5]
MODELS = ROOT / 'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--iterations', type=int, default=40,
                        help='ODE iterations; use 10 to reproduce the slipping baseline')
    parser.add_argument('--p', type=float, default=None, help='Fixture-only P gain override')
    parser.add_argument('--cube-z-offset', type=float, default=0.0,
                        help='Shift cube and its support together along world Z (metres)')
    parser.add_argument('--pad-pitch', type=float, default=0.0,
                        help='Fixture-only moving contact pad Y rotation in radians')
    parser.add_argument('--load-plugin', action='store_true')
    parser.add_argument('--carry', action='store_true')
    parser.add_argument('--sor',type=float,default=1.3)
    parser.add_argument('--solver',choices=['quick','world'],default='quick')
    args = parser.parse_args()
    if args.iterations < 1 or (args.p is not None and not 0 < args.p <= 0.1):
        parser.error('iterations must be positive and test P must be in (0, 0.1]')
    if not -0.01 <= args.cube_z_offset <= 0.01:
        parser.error('cube height offset must be within +/- 10 mm')
    if not -0.15 <= args.pad_pitch <= 0.15:
        parser.error('pad pitch must be within +/- 0.15 rad')
    if not 0 < args.sor < 2:
        parser.error('SOR must be in (0,2)')
    source = E.parse(MODELS/'so101/so101.sdf').getroot().find('model')
    sdf = E.Element('sdf', version='1.7')
    world = E.SubElement(sdf, 'world', name='default')
    if args.load_plugin or args.carry:
        E.SubElement(world,'plugin',name='fixture_load',filename='/tmp/libso101_grasp_load.so')
    physics = E.SubElement(world, 'physics', name='fixture', type='ode')
    E.SubElement(physics, 'max_step_size').text = '0.001'
    E.SubElement(physics, 'real_time_update_rate').text = '1000'
    ode = E.SubElement(physics, 'ode')
    solver = E.SubElement(ode, 'solver')
    E.SubElement(solver, 'type').text = args.solver
    E.SubElement(solver, 'iters').text = str(args.iterations)
    E.SubElement(solver, 'sor').text = str(args.sor)
    E.SubElement(world, 'gravity').text = '0 0 -9.8066'
    model = E.SubElement(world, 'model', name='gripper_fixture')
    E.SubElement(model, 'pose').text = '0 0 1 0 0 0'
    E.SubElement(model, 'self_collide').text = 'false'
    for name in ['gripper_link', 'moving_jaw_so101_v1_link']:
        link = copy.deepcopy(source.find(f"link[@name='{name}']"))
        if name == 'gripper_link':
            link.remove(link.find('pose'))
        else:
            pose=link.find("collision[@name='moving_jaw_collision']/pose")
            xyzrpy=pose.text.split(); xyzrpy[4]=str(args.pad_pitch)
            pose.text=' '.join(xyzrpy)
        model.append(link)
    model.append(copy.deepcopy(source.find("joint[@name='gripper']")))
    anchor = E.SubElement(model, 'joint', name='fixture_anchor', type='prismatic' if args.carry else 'fixed')
    E.SubElement(anchor, 'parent').text = 'world'
    E.SubElement(anchor, 'child').text = 'gripper_link'
    if args.carry:
        axis=E.SubElement(anchor,'axis')
        E.SubElement(axis,'xyz').text='0 0 1'
        limits=E.SubElement(axis,'limit')
        for tag,value in [('lower','-0.02'),('upper','0.15'),('effort','5'),('velocity','0.05')]:
            E.SubElement(limits,tag).text=value
    plugin = copy.deepcopy(source.find("plugin[@name='gripper_controller']"))
    plugin.find('initialPosition').text = '0.6'
    if args.p is not None:
        plugin.find('p').text = str(args.p)
    model.append(plugin)
    cube = copy.deepcopy(E.parse(MODELS/'test_cube/model.sdf').getroot().find('model'))
    E.SubElement(cube, 'pose').text = f'0.0025 -0.000218 {0.912+args.cube_z_offset} 0 0 0'
    world.append(cube)
    support = E.SubElement(world, 'model', name='cube_support')
    E.SubElement(support, 'static').text = 'true'
    E.SubElement(support, 'pose').text = f'0.0025 -0.000218 {0.892+args.cube_z_offset} 0 0 0'
    link = E.SubElement(support, 'link', name='link')
    for kind in ['collision', 'visual']:
        element = E.SubElement(link, kind, name=kind)
        geometry = E.SubElement(element, 'geometry')
        E.SubElement(E.SubElement(geometry, 'box'), 'size').text = '0.018 0.018 0.020'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    E.ElementTree(sdf).write(args.output, encoding='unicode', xml_declaration=True)
    print(args.output)

if __name__ == '__main__':
    main()
