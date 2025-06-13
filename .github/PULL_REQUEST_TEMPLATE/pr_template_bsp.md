# Checklist for new Board Support package or Component

- [ ] Component contains License
- [ ] Component contains README.md
- [ ] Project [README.md](../README.md) updated
- [ ] Component contains idf_component.yml file with `url` field defined
- [ ] Component was added to CI [upload job](https://github.com/espressif/esp-bsp/blob/master/.github/workflows/upload_component.yml#L17)
- [ ] New files were added to CI build job
- [ ] New BSP definitions added to [bsp_ext.py](../examples/bsp_ext.py)
- [ ] BSP was added to [SquareLine](https://github.com/espressif/esp-bsp/tree/master/SquareLine/common)
- [ ] BSP was added to [.pre-commit-config.yaml](.pre-commit-config.yaml) doxybook list
- [ ] _Optional:_ Component contains unit tests
- [ ] _Optional:_ BSP was added to Runner ([pytest.ini](pytest.ini), [conftest.py](conftest.py), [build-run-applications.yml](.github/workflows/build-run-applications.yml))
- [ ] CI passing

# Change description
_Please describe your change here_
