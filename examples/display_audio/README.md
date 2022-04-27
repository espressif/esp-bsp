| Supported Boards | Kaluga kit |
| ---------------- | ---------- |

# BSP: Display and audio example

This example demonstrates capabilities of Audio and Display board of ESP32-S2-Kaluga-Kit.

## Hardware preparation
1. Turn on all micro-switches on bottom side of Kaluga kit
2. Connect Audio and Display board to main Kaluga board
3. Populate jumper near WS2812 RGB LED
4. Connect speaker or plug headphones for audio playback
6. Turn on main power switch

## Functionality description
### Button REC
Starts recording from microphone ~3.7 seconds.
Recording check box on display will be checked for the recording time.

### Button MODE
Switches between preloaded WAV file and recorded file.
Selected file will be played with PLAY button.

### Button PLAY
Plays selected WAV file.
Either preloaded file from SPIFFS or microphone recording.
Playing check box on display will be checked for the playing time.

### Button SET
Every odd press turns on the RGB LED with random color.
Every even press turns off the RGB LED.

### Buttons VOL+/-
Increases/decreases playback volume by 5/100.
Current playback volume is depicted on display.
