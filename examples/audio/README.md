# BSP: Audio example

This example demonstrates capabilities of Audio board of ESP32-S3-Korvo-2.

## Audio codecs
- ES8311 - Audio output (speaker)
- ES7210 - Audio input (two microphones)

## Functionality description
### Button REC
Starts recording from microphone ~3.7 seconds.
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

<a href="https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=audio">
    <img alt="Try it with ESP Launchpad" src="https://espressif.github.io/esp-launchpad/assets/try_with_launchpad.png" width="250" height="70">
</a>
