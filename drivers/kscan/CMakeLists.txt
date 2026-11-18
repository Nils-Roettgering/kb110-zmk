# Copyright (c) 2026 kb110 contributors
# SPDX-License-Identifier: MIT

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_GPIO_DUPLEX kscan_gpio_duplex.c)
