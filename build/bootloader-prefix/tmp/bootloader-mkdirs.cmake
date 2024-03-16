# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Gaspar/esp/v5.2.1/esp-idf/components/bootloader/subproject"
  "E:/Repos/diplomski/espnow_test/espnow_test/build/bootloader"
  "E:/Repos/diplomski/espnow_test/espnow_test/build/bootloader-prefix"
  "E:/Repos/diplomski/espnow_test/espnow_test/build/bootloader-prefix/tmp"
  "E:/Repos/diplomski/espnow_test/espnow_test/build/bootloader-prefix/src/bootloader-stamp"
  "E:/Repos/diplomski/espnow_test/espnow_test/build/bootloader-prefix/src"
  "E:/Repos/diplomski/espnow_test/espnow_test/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/Repos/diplomski/espnow_test/espnow_test/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "E:/Repos/diplomski/espnow_test/espnow_test/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
