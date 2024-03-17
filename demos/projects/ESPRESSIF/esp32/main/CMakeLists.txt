# Copyright (c) Microsoft Corporation. All rights reserved.
# SPDX-License-Identifier: MIT

set(COMPONENT_INCLUDE_DIRS
    .
    ../config
    ""
)

idf_component_register( SRC_DIRS "."
                        INCLUDE_DIRS ${COMPONENT_INCLUDE_DIRS})