# Copyright (c) 2024 Arthur Eichelberger
# SPDX-License-Identifier: Apache-2.0

if(NOT DEFINED NUM_MOTORS)
    message(WARNING "NUM_MOTORS not provided, defaulting to 4.")
    set(NUM_MOTORS 4)
endif()

if(NOT DEFINED ESC_TYPE)
    message(WARNING "ESC_TYPE not provided, defaulting to DSHOT.")
    set(ESC_TYPE DSHOT)
endif()

if(NOT DEFINED RX_TYPE)
    message(WARNING "RX_TYPE not provided, defaulting to CRSF.")
    set(RX_TYPE CRSF)
endif()

if(NOT ${ESC_TYPE} STREQUAL DSHOT)
    message(FATAL_ERROR "ESC_TYPE=${ESC_TYPE} not supported.")
endif()

if(NOT ${RX_TYPE} STREQUAL CRSF)
    message(FATAL_ERROR "RX_TYPE=${RX_TYPE} not supported.")
endif()
