#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0

import os
import sys
import yaml
import argparse
import shutil

bsp_dir = "bsp"
bsps = {name for name in os.listdir(bsp_dir) if os.path.isdir(os.path.join(bsp_dir, name))}


def load_yaml_file(filepath):
    with open(filepath, "r") as f:
        return yaml.safe_load(f)


def save_yaml_file(filepath, data):
    with open(filepath, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


def bsp_short_name(bsp):
    return bsp.split('/')[-1]


def replace_api_link_in_readme(bsp_path):
    readme_path = os.path.join(bsp_path, "README.md")
    if not os.path.exists(readme_path):
        print(f"No README.md in {bsp_path}, skipping.")
        return
    with open(readme_path, "r", encoding="utf-8") as f:
        content = f.read()

    new_content = content.replace("API.md", "#api-reference")

    if new_content != content:
        with open(readme_path, "w", encoding="utf-8", newline="\n") as f:
            f.write(new_content)
        print(f"Updated links in {readme_path}")
    else:
        print(f"No API.md links to replace in {readme_path}")


def main():
    parser = argparse.ArgumentParser(description="Update examples for specific BSP.")
    parser.add_argument("--bsp", required=True, help="Name of the BSP component (directory under bsp/)")
    args = parser.parse_args()
    target_bsp = args.bsp
    if not target_bsp:
        print("Please set --bsp argument")
        sys.exit(1)

    print(f"Target BSP: {target_bsp}")

    bsp_path = os.path.join(bsp_dir, target_bsp)
    if not os.path.exists(bsp_path):
        print(f"No such BSP directory: {bsp_path}")
        sys.exit(1)

    # Replace API.md link in readme
    replace_api_link_in_readme(bsp_path)

    bsp_yml_path = os.path.join(bsp_path, "idf_component.yml")
    if not os.path.exists(bsp_yml_path):
        print(f"No idf_component.yml in {bsp_path}")
        sys.exit(1)

    bsp_yml = load_yaml_file(bsp_yml_path)
    examples = bsp_yml.get("examples", [])
    if not examples:
        print(f"No examples listed in {bsp_yml_path}")
        sys.exit(0)

    print(f"Configuring examples for BSP: {target_bsp}")

    for ex in examples:
        ex_rel_path = ex.get("path")
        if not ex_rel_path:
            continue
        example_abs_path = os.path.abspath(os.path.join(bsp_path, ex_rel_path))

        # idf_component.yml
        example_main_yml = os.path.join(example_abs_path, "main", "idf_component.yml")
        if not os.path.exists(example_main_yml):
            print(f"Missing {example_main_yml}, skipping.")
            continue

        manifest = load_yaml_file(example_main_yml)

        # Remove all BSPs
        for dep in list(manifest['dependencies']):
            if bsp_short_name(dep) in bsps:
                del manifest['dependencies'][dep]

        # Add the one we need
        manifest['dependencies'][target_bsp] = {
            "version": "*",
            "override_path": f"../../../{bsp_path}"
        }

        save_yaml_file(example_main_yml, manifest)
        print(f"Updated dependencies in {example_main_yml}")

        # sdkconfig.defaults
        sdkconfig_bsp = os.path.join(example_abs_path, f"sdkconfig.bsp.{target_bsp}")
        sdkconfig_dst = os.path.join(example_abs_path, "sdkconfig.defaults")
        if os.path.exists(sdkconfig_bsp):
            shutil.copyfile(sdkconfig_bsp, sdkconfig_dst)
            print(f"Copied {sdkconfig_bsp} -> {sdkconfig_dst}")
        else:
            print(f"Missing {sdkconfig_bsp}, skipping copy.")


if __name__ == "__main__":
    main()
