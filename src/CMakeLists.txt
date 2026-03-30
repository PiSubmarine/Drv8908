include("${_build_cmake_local}")

add_library(PiSubmarine.Drv8908 PiSubmarine/Drv8908/Device.cpp)
target_include_directories(PiSubmarine.Drv8908 PUBLIC "../public")
PiSubmarineInitTarget(PiSubmarine.Drv8908)

PiSubmarineAddDependency("https://github.com/PiSubmarine/RegUtils" "")
target_link_libraries(PiSubmarine.Drv8908 PUBLIC PiSubmarine.RegUtils)

PiSubmarineAddDependency("https://github.com/PiSubmarine/SPI.Api" "")
target_link_libraries(PiSubmarine.Drv8908 PUBLIC PiSubmarine.SPI.Api)

PiSubmarineAddDependency("https://github.com/PiSubmarine/GPIO.Api" "")
target_link_libraries(PiSubmarine.Drv8908 PUBLIC PiSubmarine.GPIO.Api)

PiSubmarineAddDependency("https://github.com/PiSubmarine/Units" "")
target_link_libraries(PiSubmarine.Drv8908 PUBLIC PiSubmarine.Units)