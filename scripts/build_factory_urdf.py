#!/usr/bin/env python3
"""Generate the offline UR10 body model from the saved official factory YAML."""
import argparse
from pathlib import Path


def main():
    import xacro
    from ament_index_python.packages import get_package_share_directory
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=root / 'outputs/ik/ur10_factory_20261027.urdf')
    args = parser.parse_args()
    source = Path(get_package_share_directory('ur_description')) / 'urdf/ur.urdf.xacro'
    document = xacro.process_file(str(source), mappings={
        'ur_type': 'ur10', 'name': 'ur',
        'kinematics_params': str(root / 'config/ur10_factory_calibration.yaml')})
    output = args.output if args.output.is_absolute() else root / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(document.toprettyxml(indent='  '))
    print(output)


if __name__ == '__main__':
    main()
