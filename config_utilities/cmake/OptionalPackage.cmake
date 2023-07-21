# Finds an optional dependency gated on an enable flag
#
# Sets ENABLE_${package_name} to TRUE or FALSE depending on availability. All
# results of find_package(package_name) are available if ENABLE_${package_name}
# is TRUE
#
# Args: package_name: Name of the package to find package_enabled: Whether or
# not the package should be used
macro(FIND_OPTIONAL package_name package_enabled)
  if(${package_enabled})
    find_package(${package_name})
  endif()

  if(${${package_name}_FOUND})
    set(ENABLE_${package_name} TRUE)
  else()
    set(ENABLE_${package_name} FALSE)
    if(${package_enabled})
      message(
        WARNING
          "${package_name} not found. Either disable or install the package")
    endif()
  endif()
endmacro()

# Finds an optional dependency gated on an enable flag via PkgConfig.
#
# Sets ENABLE_${package_name} to TRUE or FALSE depending on availability. The
# dependency will be available under PkgConfig::package_name
#
# Args: package_name: Name of the package to find package_enabled: Whether or
# not the package should be used
macro(FIND_OPTIONAL_PKGCFG package_name package_enabled)
  unset(${package_name}_FOUND CACHE)
  if(${package_enabled})
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(${package_name} IMPORTED_TARGET ${package_name})
  endif()

  if(${${package_name}_FOUND})
    set(ENABLE_${package_name} TRUE)
  else()
    set(ENABLE_${package_name} FALSE)
    if(${package_enabled})
      message(
        WARNING
          "${package_name} not found. Either disable or install the package")
    endif()
  endif()
endmacro()
