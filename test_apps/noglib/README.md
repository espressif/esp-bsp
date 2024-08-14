# No Graphical Library BSP test_app

BSPs are by default shipped with LVGL graphical library. This test_app shows how you can use 'noglib' version of BSPs (without LVGL) and is used mainly for CI testing purposes.

More information about noglib BSPs can be found in root [README file](../../README.md).

## How to use the example

### Hardware Required

* Any of the supported boards

### Compile and flash

This example is used mainly for CI testing purposes. If you want to build it locally, follow these steps:

1. Create a noglib version of your BSP with `python ./.github/ci/bsp_noglib.py <name_of_your_bsp>`
2. Update [idf_component.yml](main/idf_component.yml) with your BSP
3. Compile and flash as usual `idf.py -p COMx flash monitor`

### Example outputs

Following picture will be displayed with wave effect: ![](image.jpg)
