# BSP: Display example

## Overview

<table>
<tr><td valign="top">

This is a minimal display example using the LVGL graphics library.
With just a few function calls, it sets up the display and shows the Espressif logo along with a text label.

</td><td width="200" valign="top">
  <img src="/examples/display/doc/pic.webp">
</td></tr>
</table>

## Build and Flash

To build and flash the example for a specific `{board}` and `{port}`, use the following command:

```
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.{board} -p {port} flash monitor
```
Make sure the correct board name is set in the `main/idf_component.yml` file under the `dependencies` section.

## Launch Example

You can also try this example using ESP Launchpad:

<a href="https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display-">
    <img alt="Try it with ESP Launchpad" src="https://espressif.github.io/esp-launchpad/assets/try_with_launchpad.png" width="250" height="70">
</a>
