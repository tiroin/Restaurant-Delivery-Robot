# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/thevien257/.espressif/v6.0/esp-idf/components/bootloader/subproject"
  "/home/thevien257/Desktop/term/Embedded/Restaurant-Delivery-Robot/MCU1-flowchart/build/bootloader"
  "/home/thevien257/Desktop/term/Embedded/Restaurant-Delivery-Robot/MCU1-flowchart/build/bootloader-prefix"
  "/home/thevien257/Desktop/term/Embedded/Restaurant-Delivery-Robot/MCU1-flowchart/build/bootloader-prefix/tmp"
  "/home/thevien257/Desktop/term/Embedded/Restaurant-Delivery-Robot/MCU1-flowchart/build/bootloader-prefix/src/bootloader-stamp"
  "/home/thevien257/Desktop/term/Embedded/Restaurant-Delivery-Robot/MCU1-flowchart/build/bootloader-prefix/src"
  "/home/thevien257/Desktop/term/Embedded/Restaurant-Delivery-Robot/MCU1-flowchart/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/thevien257/Desktop/term/Embedded/Restaurant-Delivery-Robot/MCU1-flowchart/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/thevien257/Desktop/term/Embedded/Restaurant-Delivery-Robot/MCU1-flowchart/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
