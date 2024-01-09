# Contributing

Contributions to ESP-BSP project in the form of pull requests, bug reports, and feature requests are welcome!

This document covers various topics related to contributions to the ESP-BSP projects. Please read it if you plan to submit a PR!

## CLA

We require accepting the contributor's license agreement for all pull requests. When opening a pull request the first time you will be prompted to sign the CLA by the [CLA Assistant](https://cla-assistant.io/) service.

## Large-scale changes

If you'd like to propose a change to the existing APIs or a large-scale refactoring of the implementation, we recommend opening an issue first to discuss this.

## Third-party boards

ESP-BSP project is currently intended only to host BSPs for development boards manufactured by Espressif.

If you want to create a BSP for a third-party board, we suggest creating a separate repository for it. You are welcome to use the ESP-BSP project as a template for your own board support package repository.

## Pre-commit hooks

ESP-BSP project uses [pre-commit hooks](https://pre-commit.com/) to perform code formatting and other checks when you run `git commit`.

To install pre-commit hooks, run `pip install pre-commit && pre-commit install`. 

If a pre-commit hook has modified any of the files when you run `git commit`, add these changes using `git add` and run `git commit` again.

## Adding new components

[Pull request template](.github/PULL_REQUEST_TEMPLATE/pr_template_bsp.md) contains a checklist of things we require in new components. Please familiarize yourself with the checklist when developing a new component.

New Board Support Packages should follow recommendations in [BSP development guide](./BSP_development_guide.md).

## Supported IDF versions

All components are expected to be usable with multiple supported IDF versions. You can find the list in the [CI workflow file](.github/workflows/build_test.yml).

