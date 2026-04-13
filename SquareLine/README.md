# SquareLine board generator

This is a simple python generator script for SquareLine board packages. This script copies all files from `common` folder to the destination ZIP file. It also generates other files according to the `manifest.json` file from each board directory in `boards` folder. For information on the structure of SquareLine board packages, please refer to [Open Board Platform (OBP)](https://docs.squareline.io/docs/obp/) documentation.

## Mandatory files in each board folder:
* `manifest.json` - Description of the board
* `image.png` - Image of the board 380px x 300px
* `sdkconfig.defaults` - defaults
* `main/idf_component.yml` - Component Registry description

## Optional files and folders in each board folder:
* `components` - Folder with components (will be copied without using placeholders)

### `manifest.json`

Mandatory JSON keys in manifest:
* `name` - Board name (string)
* `version` - Package version (string)
* `mcu` - ESP mcu (string)
* `screen_width` - Screen width (string)
* `screen_height` - Screen height (string)
* `screen_color_swap` - Is color bytes swapped (boolean)
* `short_description` - Short board description (string)
* `long_description` - Long board description (string)
* `placeholders` - List of placeholders and values of them (JSON object)

### Placeholders

When copying all the files (except for `image.png`), the script will replace placeholders found in these files. Placeholder must be in compound brackets (e.g. `{PLACEHOLDER}`). The placeholders and their values should be defined in `manifest.json` as follows:

```
    "placeholders":
    {
        "PLACEHOLDER_1": "value 1",
        "PLACEHOLDER_2": "value 2"
    }
```

**_NOTE 1:_** Placeholders are optional. But when some placeholder is found in code and not found in JSON, it will not be substituted.

**_NOTE 2:_** All default values (SLB file, default folders, etc...) are saved in `gen.py`.

# Usage

Generate all available boards to `out_dir` folder

```
    python gen.py -o out_dir
```

Generate only one selected board `board_dir` to `out_dir` folder

```
    python gen.py -o out_dir -b board_dir
```

**_NOTE:_** The output folder is cleaned before generating.

# Custom Boards Usage

The generator supports custom boards. The example of the custom board is [here](boards/custom_waveshare_7inch/). There is custom BSP for the custom board as a component. For use this custom board in SquareLine, follow these steps:

1. Generate custom board.
```
    python gen.py -o out_dir -b custom_waveshare_7inch
```

2. Copy folder from `out_dir/espressif/custom_waveshare_7inch` to `"SquareLine Studio installation path"/boards/espressif/`
