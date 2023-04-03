import os
from re import match, compile
from pathlib import Path
from click.core import Context
from typing import List
from idf_py_actions.tools import PropertyDict, red_print


def action_extensions(base_actions, project_path=os.getcwd()):
    """ Describes extension for Board Support Packages. """

    try:
        import ruamel.yaml
    except ImportError:
        red_print('ruamel.yaml package is not installed. No BSP extension is added!')
        return {}

    bsp_sdkconfig_regex = compile('^SDKCONFIG_DEFAULTS=.*sdkconfig\.bsp\.')

    def global_callback(ctx: Context, global_args: PropertyDict, tasks: List) -> None:
        # In case the user has defined his own BSP configuration, run set-bsp action before anything else
        for entry in global_args['define_cache_entry']:
            if match(bsp_sdkconfig_regex, entry):
                tasks.insert(0, ctx.invoke(ctx.command.get_command(ctx, 'set-bsp')))
                return

    def bsp_short_name(bsp):
        return bsp.split('/')[-1]

    def set_bsp_callback(action: str, ctx: Context, args: PropertyDict, **kwargs: str) -> None:
        # Find configuration name
        bsp = ''
        for entry in args['define_cache_entry']:
            if match(bsp_sdkconfig_regex, entry):
                bsp = entry.split('.')[-1]
                break

        # List of supported BSPs
        bsps = {
            'esp_wrover_kit',
            'esp32_azure_iot_kit',
            'esp32_s2_kaluga_kit',
            'esp32_s3_eye',
            'esp32_s3_lcd_ev_board',
            'esp32_s3_usb_otg',
            'esp-box',
            'esp32_s3_korvo_2',
            'esp-box-lite',
            }

        if bsp == '': return
        if bsp not in bsps:
            print("Invalid BSP configuration " + bsp)
            return

        print('Setting project for BSP: ' + bsp)
        manifest_path  = Path(args['project_dir']) / 'main' / 'idf_component.yml'
        yaml = ruamel.yaml.YAML()
        manifest = yaml.load(manifest_path)

        # Remove all BSPs
        for dep in list(manifest['dependencies']):
            if bsp_short_name(dep) in bsps:
                del manifest['dependencies'][dep]

        # Add the one we need
        manifest['dependencies'].insert(0, bsp, {'version': '*', 'override_path': ('../../../' + bsp_short_name(bsp))})
        yaml.dump(manifest, manifest_path)

    extensions = {
        'global_action_callbacks': [global_callback],
        'actions': {
            'set-bsp': {
                'callback': set_bsp_callback,
                'help': 'Utility to set Board Support Package for a project.',
                'hidden': True,
                'options': [],
                'order_dependencies': [],
                'arguments': [],
            },
        },
    }

    return extensions
