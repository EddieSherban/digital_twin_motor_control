# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/esp/esp-idf/components/bootloader/subproject"
  "D:/Documents/Electrical Egneinerineign/Electrical Engineering/Projects/test/DigitalTwinCapstone/build/bootloader"
  "D:/Documents/Electrical Egneinerineign/Electrical Engineering/Projects/test/DigitalTwinCapstone/build/bootloader-prefix"
  "D:/Documents/Electrical Egneinerineign/Electrical Engineering/Projects/test/DigitalTwinCapstone/build/bootloader-prefix/tmp"
  "D:/Documents/Electrical Egneinerineign/Electrical Engineering/Projects/test/DigitalTwinCapstone/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Documents/Electrical Egneinerineign/Electrical Engineering/Projects/test/DigitalTwinCapstone/build/bootloader-prefix/src"
  "D:/Documents/Electrical Egneinerineign/Electrical Engineering/Projects/test/DigitalTwinCapstone/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Documents/Electrical Egneinerineign/Electrical Engineering/Projects/test/DigitalTwinCapstone/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Documents/Electrical Egneinerineign/Electrical Engineering/Projects/test/DigitalTwinCapstone/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
