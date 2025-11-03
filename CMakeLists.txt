# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(include)
#add_subdirectory(driver)
zephyr_library()
zephyr_library_sources(driver/em32f967_hal.c)
