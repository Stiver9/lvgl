# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Stiver/esp/esp-idf/components/bootloader/subproject"
  "C:/Projects/lvgl/build/bootloader"
  "C:/Projects/lvgl/build/bootloader-prefix"
  "C:/Projects/lvgl/build/bootloader-prefix/tmp"
  "C:/Projects/lvgl/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Projects/lvgl/build/bootloader-prefix/src"
  "C:/Projects/lvgl/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Projects/lvgl/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
