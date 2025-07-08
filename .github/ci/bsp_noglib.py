#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0

"""
Remove esp_lvgl_port from BSP's dependencies
"""

import sys
import argparse
from pathlib import Path
from idf_component_tools.manifest import ManifestManager
from update_readme_dependencies import check_bsp_readme

DEFINE_NOGLIB_OFF = '#define BSP_CONFIG_NO_GRAPHIC_LIB (0)'
DEFINE_NOGLIB_ON = '#define BSP_CONFIG_NO_GRAPHIC_LIB (1)'
ESP_REGISTRY_URL = 'https://components.espressif.com/components/'
README_NOGLIB_NOTICE = '> :warning: This is **No Graphical Library version** of {} BSP. If you want to use this BSP with LVGL use [{}]({}) component.\n'


def select_bsp_config_no_graphic_lib(bsp_path):
    config_path = bsp_path / 'include' / 'bsp' / 'config.h'
    try:
        with open(config_path, encoding='utf-8', mode='r') as header:
            content = header.read()
        content = content.replace(DEFINE_NOGLIB_OFF, DEFINE_NOGLIB_ON)
        with open(config_path, encoding='utf-8', mode='w', newline='\n') as header:
            header.write(content)
        return 0
    except FileNotFoundError:
        print("{}: could not modify config.h".format(str(bsp_path)))
        return 1


def remove_esp_lvgl_port(bsp_path):
    manager = ManifestManager(bsp_path, 'bsp')
    try:
        del manager.manifest_tree["dependencies"]["espressif/esp_lvgl_port"]
    except KeyError:
        print("{}: could not remove esp_lvgl_port".format(str(bsp_path)))
        return 1
    try:
        del manager.manifest_tree["dependencies"]["lvgl/lvgl"]
    except KeyError:
        print("{}: no lvgl dependency found".format(str(bsp_path)))
    manager.manifest_tree["description"] = manager.manifest_tree["description"] + ' with no graphical library'

    # Add 'noglib' tag
    tags = manager.manifest_tree.get("tags", [])
    tags.append("noglib")
    manager.manifest_tree["tags"] = list(set(tags))  # make unique

    manager.dump()
    return 0


def add_notice_to_readme(bsp_path):
    readme_path = bsp_path / 'README.md'
    bsp_name = bsp_path.parts[-1].rstrip("_noglib")
    try:
        with open(readme_path, encoding='utf-8', mode='r') as readme:
            content = readme.readlines()

        content.insert(0, README_NOGLIB_NOTICE.format(bsp_name, bsp_name, ESP_REGISTRY_URL + 'espressif/' + bsp_name))
        with open(readme_path, encoding='utf-8', mode='w', newline='\n') as readme:
            readme.writelines(content)
        return 0
    except FileNotFoundError:
        print("{}: could not modify README.md".format(bsp_name))
        return 1


def bsp_no_glib_all(bsp_names):
    ret = 0
    for bsp in bsp_names:
        bsp_path = Path(bsp)
        if not bsp_path.exists():
            print("[Error] Argument {} does not point to existing BSP".format(bsp))
            raise Exception

        # 1. Rename the BSP to BSP_noglib
        bsp_path = bsp_path.rename(bsp + "_noglib")

        # 2. Modify the configuration, dependencies and README
        ret += select_bsp_config_no_graphic_lib(bsp_path)
        ret += remove_esp_lvgl_port(bsp_path)
        ret += add_notice_to_readme(bsp_path)
        try:
            check_bsp_readme(bsp_path)
        except Exception:
            pass  # Exception here means that the README.md was modified, which is OK here
    return ret


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bsp_names', nargs='*', help='BSP names to process')
    args = parser.parse_args()
    sys.exit(bsp_no_glib_all(args.bsp_names))
