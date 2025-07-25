include(GNUInstallDirs)
install(
  TARGETS ${PROJECT_NAME} composite-configs
  EXPORT config_utilities-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(
  EXPORT config_utilities-targets
  FILE config_utilitiesTargets.cmake
  NAMESPACE config_utilities::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/config_utilities)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/config_utilitiesConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/config_utilitiesConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/config_utilities)
write_basic_package_version_file(
  config_utilitiesConfigVersion.cmake
  VERSION ${PACKAGE_VERSION}
  COMPATIBILITY AnyNewerVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/config_utilitiesConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/config_utilitiesConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/config_utilities)

find_package(ament_cmake_core QUIET)
if (${ament_cmake_core_FOUND})
  ament_package()
endif()
