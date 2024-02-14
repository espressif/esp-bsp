#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0

"""
Remove esp_lvgl_port from BSP's dependencies
"""

import sys
from pathlib import Path
from idf_component_tools.manifest import ManifestManager

DEFINE_NOGLIB_OFF = '#define BSP_CONFIG_NO_GRAPHIC_LIB (0)'
DEFINE_NOGLIB_ON = '#define BSP_CONFIG_NO_GRAPHIC_LIB (1)'
ESP_REGISTRY_URL = 'https://components.espressif.com/components/'
README_NOGLIB_NOTICE = '> :warning: This is **No Graphical version** of {} BSP. If you want to use this BSP with LVGL use [{}]({}) component.\n'


def select_bsp_config_no_graphic_lib(bsp_name):
    config_path = Path('bsp') / bsp_name / 'include' / 'bsp' / 'config.h'
    try:
        with open(config_path, encoding='utf-8', mode='r') as header:
            content = header.read()
        content = content.replace(DEFINE_NOGLIB_OFF, DEFINE_NOGLIB_ON)
        with open(config_path, encoding='utf-8', mode='w') as header:
            header.write(content)
        return 0
    except FileNotFoundError:
        print("{}: could not modify config.h".format(bsp_name))
        return 1


def remove_esp_lvgl_port(bsp_name):
    bsp_path = Path('bsp') / bsp_name
    manager = ManifestManager(bsp_path, 'bsp')
    try:
        del manager.manifest_tree["dependencies"]["espressif/esp_lvgl_port"]
    except KeyError:
        print("{}: could not remove esp_lvgl_port".format(bsp_name))
        return 1
    try:
        del manager.manifest_tree["dependencies"]["lvgl/lvgl"]
    except KeyError:
        print("{}: no lvgl dependency found".format(bsp_name))
    manager.manifest_tree["description"] = manager.manifest_tree["description"] + ' with no graphical library'
    manager.dump()
    return 0


def add_notice_to_readme(bsp_name):
    readme_path = Path('bsp') / bsp_name / 'README.md'
    try:
        with open(readme_path, encoding='utf-8', mode='r') as readme:
            content = readme.readlines()

        content.insert(0, README_NOGLIB_NOTICE.format(bsp_name, bsp_name, ESP_REGISTRY_URL + 'espressif/' + bsp_name))
        with open(readme_path, encoding='utf-8', mode='w') as readme:
            readme.writelines(content)
        return 0
    except FileNotFoundError:
        print("{}: could not modify README.md".format(bsp_name))
        return 1


def bsp_no_glib_all():
    # First version modifies only ESP32-S3-EYE BSP
    # TODO: Add a loop that will process all BSPs
    ret = select_bsp_config_no_graphic_lib("esp32_s3_eye")
    ret += remove_esp_lvgl_port("esp32_s3_eye")
    ret += add_notice_to_readme("esp32_s3_eye")
    return ret


if __name__ == '__main__':
    sys.exit(bsp_no_glib_all())
