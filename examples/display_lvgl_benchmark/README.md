# BSP: Display LVGL Benchmark

This example runs the LVGL benchmark demo to measure graphical performance on Espressif and M5Stack boards. It is used in CI for selected pull requests (based on labels) and after merging changes into the master branch.

## Main Features
- Can be triggered by adding the "Run benchmark" label to a PR.
- The measured values in a PR are compared against the master branch and posted as a comment, highlighting any differences.
- Benchmark results for the master branch are stored in BSP releases.

## How to use the example

This example can be used as standalone example too.

### Hardware Required

* ESP32-S3-LCD-EV-Board or ESP32-S3-LCD-EV-Board-2
* USB-C Cable

## Build and Flash

To build and flash the example for a specific `{board}` and `{port}`, use the following command:

```
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.{board} -p {port} flash monitor
```
Make sure the correct board name is set in the `main/idf_component.yml` file under the `dependencies` section.


