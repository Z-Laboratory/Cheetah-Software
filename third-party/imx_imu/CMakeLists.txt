set(IS_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../inertial-sense-sdk")

find_package(Threads REQUIRED)

include(${IS_SDK_DIR}/include_is_sdk_find_library.cmake)

add_library(imx_imu
    ImxImu.cpp
)

target_include_directories(imx_imu PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${IS_SDK_DIR}/src
    ${IS_SDK_DIR}/src/libusb/libusb
    ${CMAKE_SOURCE_DIR}/common/include
)

target_link_libraries(imx_imu
    PUBLIC
    InertialSenseSDK
    Threads::Threads
)

set_property(TARGET imx_imu PROPERTY CXX_STANDARD 20)
set_property(TARGET imx_imu PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(imx_cheetah_test
    test_imx.cpp
)

target_include_directories(imx_cheetah_test PRIVATE
    ${CMAKE_SOURCE_DIR}/robot/include
    ${CMAKE_SOURCE_DIR}/third-party/vectornav/include
)

target_link_libraries(imx_cheetah_test
    PRIVATE
    imx_imu
)

set_property(TARGET imx_cheetah_test PROPERTY CXX_STANDARD 17)
set_property(TARGET imx_cheetah_test PROPERTY CXX_STANDARD_REQUIRED ON)