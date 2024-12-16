# Copyright (c) 2024 Arthur Eichelberger
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_RC_LINK_SHELL rc_link_shell.c)
zephyr_library_sources_ifdef(CONFIG_RC_LINK_CRSF rc_link_crsf.c)
