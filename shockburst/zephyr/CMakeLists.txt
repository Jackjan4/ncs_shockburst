if (CONFIG_SHOCKBURST)

  # Add shockburst.h to the set of global include paths.
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    shockburst.c
    )


endif()