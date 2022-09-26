# BSP: Custom board for SquareLine Studio - WaveShare 7inch LCD 

This is an example how to make custom BSP and generate custom board for SquareLine Studio. Follow these steps to modify this example for your board.

## Steps to add custom board into SquareLine Studio

1. Make BSP of your board ([example](components/ws_7inch))
2. Update your BSP component in [YML](main/idf_component.yml) file.
3. Prepare `image.png` with size 380px x 300px
4. Update `manifest.json` with right name, screen size and description.
5. Generate board for SquareLine Studio
```
    python gen.py -b custom_waveshare_7inch -o output_folder
```
6. Copy generated `espressif/custom_waveshare_7inch` folder into `boards` in SquareLine Studio installation folder.
7. Launch SquareLine Studio
