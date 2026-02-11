#!/usr/bin/env python3
import sys
import yaml
from collections import defaultdict
from pathlib import Path

def find_components():
    components_dic = defaultdict(lambda: {'component_registry': []})
    for yaml_file in Path('.').rglob('bsp/*/idf_component.yml'):
        bsp = str(yaml_file).split('/')[1]  # TODO: Temporary
        with open(yaml_file) as f:
            data = yaml.safe_load(f)
        try:
            for dependency in data['dependencies']:
                if 'override_paths' in dependency:
                    continue # TODO
                else:
                    print(dependency)
                    components_dic[bsp]['component_registry'].append(dependency)
        except:
            continue
    return components_dic

def main(argv=None):
    components = find_components()
    for bsp, data in components.items():
        print(f"\n{bsp}:")
        for component in data['component_registry']:
            print(f"  {component}")

    return 0

if __name__ == "__main__":
    sys.exit(main())