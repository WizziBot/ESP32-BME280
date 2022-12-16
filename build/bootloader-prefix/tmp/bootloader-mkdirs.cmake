# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/wizzi/esp-idf/components/bootloader/subproject"
  "/home/wizzi/esp-idf/projects/BME280/build/bootloader"
  "/home/wizzi/esp-idf/projects/BME280/build/bootloader-prefix"
  "/home/wizzi/esp-idf/projects/BME280/build/bootloader-prefix/tmp"
  "/home/wizzi/esp-idf/projects/BME280/build/bootloader-prefix/src/bootloader-stamp"
  "/home/wizzi/esp-idf/projects/BME280/build/bootloader-prefix/src"
  "/home/wizzi/esp-idf/projects/BME280/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/wizzi/esp-idf/projects/BME280/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/wizzi/esp-idf/projects/BME280/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
