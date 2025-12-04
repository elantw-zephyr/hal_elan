# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_EM32F967)
  zephyr_include_directories(include)
  #add_subdirectory(driver)
  zephyr_library()
  zephyr_library_sources(driver/em32f967_hal.c)
endif()
