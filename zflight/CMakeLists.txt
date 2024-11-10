# zflight application
#
# Copyright (c) 2024 Arthur Eichelberger
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

# Check the required build args were supplied
if (NOT DEFINED NUM_MOTORS)
    message(FATAL_ERROR "NUM_MOTORS not set")
else()
    set(ENV{NUM_MOTORS} ${NUM_MOTORS})
endif()
if (NOT DEFINED ESC_TYPE)
    message(FATAL_ERROR "ESC_TYPE not set")
endif()

# Set important directories
set(WORKSPACE_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
set(ENV{ZEPHYR_BASE} ${WORKSPACE_DIR}/zephyr)
set(ZEPHYR_SDK_INSTALL_DIR ${WORKSPACE_DIR}/zephyr-sdk)
set(APP_DIR ${WORKSPACE_DIR}/app)
set(ZFLIGHT_DIR ${APP_DIR}/zflight)

# Set snippets from inputs
set(ESC_TYPE_SNIPPET esc-${ESC_TYPE})
list(APPEND SNIPPET ${ESC_TYPE_SNIPPET})

# Configure Zephyr
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zflight LANGUAGES C)

# Configure zflight source
target_include_directories(app PRIVATE include)
file(GLOB_RECURSE SRCS src/*.c)
target_sources(app PRIVATE ${SRCS})
