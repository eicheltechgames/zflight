# Copyright (c) 2024 Arthur Eichelberger
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(dshot_common.c)
zephyr_library_sources_ifdef(CONFIG_DSHOT_STM32	dshot_stm32.c)
zephyr_library_sources_ifdef(CONFIG_DSHOT_SHELL dshot_shell.c)
