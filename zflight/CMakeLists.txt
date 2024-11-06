# zflight application
#
# Copyright (c) 2024 Arthur Eichelberger
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

set(WORKSPACE_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
set(ENV{ZEPHYR_BASE} ${WORKSPACE_DIR}/zephyr)
set(ENV{ZEPHYR_SDK_INSTALL_DIR} ${WORKSPACE_DIR}/zephyr-sdk)
set(ENV{APP_DIR} ${WORKSPACE_DIR}/app)
set(ENV{ZFLIGHT_DIR} ${APP_DIR}/zflight)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
include(verify_args)
set(ENV{NUM_MOTORS} ${NUM_MOTORS})
set(ENV{ESC_TYPE} ${ESC_TYPE})
set(ENV{RX_TYPE} ${RX_TYPE})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

target_sources(app PRIVATE src/main.c)
