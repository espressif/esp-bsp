import os
from pathlib import Path


def action_extensions(base_actions, project_path=os.getcwd()):
    """ Describes extension for Board Support Packages. """

    try:
        import ruamel.yaml
    except ImportError:
        print("!! ruamel.yaml package is not installed. No BSP extension is added !!")
        return -1

    def bsp_short_name(bsp):
        return bsp.split('/')[-1]

    def set_bsp_callback(action: str, ctx, args, **kwargs: str) -> None:
        yaml = ruamel.yaml.YAML()
        # List of supported BSPs and their targets
        # Don't forget to update this dictionary and build_example_for_all_bsps.sh when adding new BSP
        bsps = {
            'esp_wrover_kit'       : 'esp32',
            'esp32_azure_iot_kit'  : 'esp32',
            'esp32_s2_kaluga_kit'  : 'esp32s2',
            'esp32_s3_eye'         : 'esp32s3',
            'esp32_s3_lcd_ev_board': 'esp32s3',
            'esp32_s3_usb_otg'     : 'esp32s3',
            'esp-box'              : 'esp32s3',
            'esp32_s3_korvo_2'     : 'esp32s3',
            }
        manifest_path  = Path(project_path) / 'main' / 'idf_component.yml'
        manifest = yaml.load(manifest_path)

        # Remove all BSPs
        for dep in list(manifest['dependencies']):
            if bsp_short_name(dep) in bsps:
                del manifest['dependencies'][dep]

        # Add one we need
        manifest['dependencies'].insert(0, kwargs['bsp'], {'version': '*', 'override_path': ('../../../' + bsp_short_name(kwargs['bsp']))})
        yaml.dump(manifest, manifest_path)

        # Set-target according to the BSP
        base_actions['actions']['set-target']['callback']('set-target', ctx, args, bsps[bsp_short_name(kwargs['bsp'])] )

    extensions = {
        'actions': {
            'set-bsp': {
                'callback': set_bsp_callback,
                'help': 'Utility to set Board Support Package for a project.',
                'options': [],
                'order_dependencies': [],
                'arguments': [
                    {
                        'names': ['bsp'],
                        'required': True,
                    },
                ],
            },
        },
    }

    return extensions
