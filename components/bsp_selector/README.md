# BSP Selector

This component allows to select BSP in project from `menuconfig`. It shows only BSPs available for selected target.

## Usage

Add this component into your project `idf_component.yml` file like this:

```
...
dependencies:
  bsp_selector: "^1.0"

```

Select target:
```
idf.py set-target esp32s3
```

Select BSP in menuconfig:
```
idf.py menuconfig
```

`Component config` -> `BSP Selector` -> `Select BSP`

> [!NOTE]
> Only BSPs compatible with the selected target are available for selection.

Build, flash and start monitor:
```
idf.py flash monitor
```
