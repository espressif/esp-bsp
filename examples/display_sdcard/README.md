# Display and SD card example

This example shows how you can interact with a display and perform reads and writes to an SD card.

By default, the SD card is interfaced using the SDMMC inteface.

### Hardware Required

Any compatible board and an SD card.

## Build and Flash

To build and flash the example for a specific `{board}` and `{port}`, use the following command:

```
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.{board} -p {port} flash monitor
```
Make sure the correct board name is set in the `main/idf_component.yml` file under the `dependencies` section.

