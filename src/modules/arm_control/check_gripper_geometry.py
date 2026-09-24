#!/usr/bin/env python3
"""Read-only binary STL bounds audit; does not prove contact or grasp stability."""
from pathlib import Path
import struct
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
ASSETS = ROOT / 'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/so101/assets'
SPECS = [
    ('moving_jaw_so101_v1.stl', np.array([[1,0,0],[0,0,-1],[0,1,0]]), [0,-.0189,0]),
    ('wrist_roll_follower_so101_v1.stl', np.diag([1,-1,-1]), [0,-.000218,.00095]),
]

def main():
    # Visual transforms from current SDF; rounded ideal quarter/half turns.
    for name, rotation, translation in SPECS:
        data = (ASSETS / name).read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        assert len(data) == 84 + count * 50, 'Expected binary STL'
        points = np.array([struct.unpack_from('<12f', data, 84+i*50)[3:]
                           for i in range(count)]).reshape(-1,3)
        points = points @ rotation.T + translation
        print(name, 'metres in its link frame')
        print('bounds:', points.min(0), points.max(0))
        for z in [-.04, -.05, -.06, -.07]:
            distal = points[points[:,2] < z]
            if len(distal):
                print('z <', z, ':', distal.min(0), distal.max(0))

if __name__ == '__main__':
    main()
