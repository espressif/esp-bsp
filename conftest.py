# SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
import pytest


# Check marker with params and test ID
def pytest_collection_modifyitems(config, items):
    for item in items:
        marker_option = "[" + config.getoption("-m") + "]"
        if marker_option not in item.nodeid:
            item.add_marker(pytest.mark.skip(reason="Not for selected params"))
