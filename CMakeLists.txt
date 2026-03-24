cmake_minimum_required(VERSION 3.16)

# Set the target chip — must be before project()
# Override on command line: idf.py set-target esp32s3
if(NOT DEFINED IDF_TARGET)
    set(IDF_TARGET "esp32s3")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(smart_neckband)
