cmake_minimum_required(VERSION 3.20)

include(FetchContent)

project(PiSubmarine.Drv8908.Test LANGUAGES CXX)

set(PiSubmarine.Drv8908.Test.Sources 
	"PiSubmarine/Drv8908/Drv8908Test.cpp")

enable_testing()

# spdlog
find_package(spdlog)
find_package(gtest)

add_executable(PiSubmarine.Drv8908.Test ${PiSubmarine.Drv8908.Test.Sources})
target_compile_features(PiSubmarine.Drv8908.Test PRIVATE cxx_std_23)
target_link_libraries(PiSubmarine.Drv8908.Test PRIVATE PiSubmarine.Drv8908)
target_link_libraries(PiSubmarine.Drv8908.Test PRIVATE spdlog::spdlog)
target_link_libraries(PiSubmarine.Drv8908.Test PRIVATE GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(PiSubmarine.Drv8908.Test)