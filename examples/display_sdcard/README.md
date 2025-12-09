# Display and SD card example

<table>
<tr><td valign="top">

This example shows how you can interact with a display and perform reads and writes to an SD card.
By default, the SD card is interfaced using the SDMMC inteface.

</td><td width="200" valign="top">
  <img src="/examples/display_sdcard/doc/pic.webp">
</td></tr>
</table>

### Hardware Required

Any compatible board and an SD card.

## Build and Flash

To build and flash the example for a specific `{board}` and `{port}`, use the following command:

```
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.{board} -p {port} flash monitor
```
Make sure the correct board name is set in the `main/idf_component.yml` file under the `dependencies` section.

