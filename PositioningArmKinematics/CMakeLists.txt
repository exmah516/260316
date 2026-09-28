cmake_minimum_required(VERSION 3.24)
project(PositioningArmKinematics VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

add_library(PositioningArmKinematics STATIC src/kinematics.cpp src/program_zero.cpp)
add_library(PositioningArm::Kinematics ALIAS PositioningArmKinematics)
target_compile_features(PositioningArmKinematics PUBLIC cxx_std_17)
target_include_directories(PositioningArmKinematics
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
target_link_libraries(PositioningArmKinematics PUBLIC Eigen3::Eigen)
if(MSVC)
    target_compile_options(PositioningArmKinematics PRIVATE /utf-8 /bigobj /W4)
    target_compile_definitions(PositioningArmKinematics PRIVATE NOMINMAX)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(positioning_arm_tests tests/kinematics_test.cpp)
    target_link_libraries(positioning_arm_tests PRIVATE PositioningArmKinematics)
    if(MSVC)
        target_compile_options(positioning_arm_tests PRIVATE /utf-8 /W4)
    endif()
    add_test(NAME positioning_arm_tests COMMAND positioning_arm_tests)
    if(WIN32)
        set(ADS_PROJECT "${CMAKE_CURRENT_SOURCE_DIR}/../64位ADS - 相对路径 - 传数组 - 加上手柄")
        add_executable(arm_ads_tests tests/arm_ads_test.cpp "${ADS_PROJECT}/arm_manual_ads_service.cpp")
        target_include_directories(arm_ads_tests PRIVATE "${ADS_PROJECT}" "${ADS_PROJECT}/ADS/Include")
        target_link_libraries(arm_ads_tests PRIVATE PositioningArmKinematics)
        target_compile_options(arm_ads_tests PRIVATE /utf-8 /bigobj)
        target_compile_definitions(arm_ads_tests PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
        add_test(NAME arm_ads_tests COMMAND arm_ads_tests)
    endif()
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/PositioningArmKinematicsConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/PositioningArmKinematicsConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PositioningArmKinematics)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/PositioningArmKinematicsConfigVersion.cmake"
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(TARGETS PositioningArmKinematics EXPORT PositioningArmTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT PositioningArmTargets NAMESPACE PositioningArm::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PositioningArmKinematics)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/PositioningArmKinematicsConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/PositioningArmKinematicsConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PositioningArmKinematics)
