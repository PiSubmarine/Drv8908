include("${_build_cmake_local}")

add_library(PiSubmarine.Drv8908 INTERFACE)
target_include_directories(PiSubmarine.Drv8908 INTERFACE "../public")

PiSubmarineAddDependency("https://github.com/PiSubmarine/RegUtils" "")
target_link_libraries(PiSubmarine.Drv8908 INTERFACE PiSubmarine.RegUtils)

PiSubmarineAddDependency("https://github.com/PiSubmarine/Api.Internal.SPI" "")
target_link_libraries(PiSubmarine.Drv8908 INTERFACE PiSubmarine.Api.Internal.SPI)