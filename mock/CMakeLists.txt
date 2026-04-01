message("${_build_cmake_local}")
include("${_build_cmake_local}")

add_library(${PISUBMARINE_PROJECT_NAME}.Mock INTERFACE
        PiSubmarine/Drv8908/IDriverMock.h
)
target_include_directories(${PISUBMARINE_PROJECT_NAME}.Mock INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
PiSubmarineInitTarget(${PISUBMARINE_PROJECT_NAME}.Mock)

find_package(GTest)
target_link_libraries(${PISUBMARINE_PROJECT_NAME}.Mock INTERFACE GTest::gtest_main GTest::gmock_main)