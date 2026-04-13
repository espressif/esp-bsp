# BSP: Audio example

This example demonstrates capabilities of Audio boards.

## Functionality description
### Button REC
Starts recording from microphone ~5.12 seconds.
Recording check box on display will be checked for the recording time.

### Button SET
Switches between preloaded WAV file and recorded file.
Selected file will be played with PLAY button.

### Button PLAY
Plays selected WAV file.
Either preloaded file from SPIFFS or microphone recording.
Playing check box on display will be checked for the playing time.

### Buttons VOL+/-
Increases/decreases playback volume by 5/100.
Current playback volume is depicted on display.

## Build and Flash

To build and flash the example for a specific `{board}` and `{port}`, use the following command:

```
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.{board} -p {port} flash monitor
```
Make sure the correct board name is set in the `main/idf_component.yml` file under the `dependencies` section.

## Launch Example

You can also try this example using ESP Launchpad:

<a href="https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=audio-">
    <img alt="Try it with ESP Launchpad" src="https://espressif.github.io/esp-launchpad/assets/try_with_launchpad.png" width="250" height="70">
</a>
