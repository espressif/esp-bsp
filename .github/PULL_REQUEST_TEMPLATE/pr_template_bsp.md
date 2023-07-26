# Checklist for new Board Support package or Component

- [ ] Component contains License
- [ ] Component contains README.md
- [ ] Project [README.md](../README.md) updated
- [ ] Component contains idf_component.yml file with `url` field defined
- [ ] Component was added to CI [upload job](https://github.com/espressif/esp-bsp/blob/master/.github/workflows/upload_component.yml#L17)
- [ ] New files were added to CI build job
- [ ] New BSP definitions added to [bsp_ext.py](../examples/bsp_ext.py)
- [ ] BSP was [added to SquareLine](https://github.com/espressif/esp-bsp/tree/master/SquareLine/common)
- [ ] New BSP supports IDF v4.4.x and later releases
- [ ] _Optional:_ Component contains unit tests
- [ ] CI passing

# Change description
_Please describe your change here_
