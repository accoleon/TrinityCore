# This file is part of the TrinityCore Project. See AUTHORS file for Copyright information
#
# This file is free software; as a special exception the author gives
# unlimited permission to copy and/or distribute it, with or without
# modifications, as long as this notice is preserved.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY, to the extent permitted by law; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

# EXPERIMENT (item 535): mmaps_generator + the mmaps_common Generator/ sources compiled into ONE
# executable, so optimisations can be tried without touching the copy the worldserver links
# (DynamicMMapTileBuilder uses mmaps_common). Isolation: this target does NOT link mmaps_common, so
# no Generator object or symbol is shared with it; the copied sources keep their MMAP namespace.
# MMapDefines.h is the verbatim header (header-only), Generator/ the verbatim sources.
add_executable(mmaps_generator_fast)

CollectAndAddSourceFiles(
  mmaps_generator_fast
  ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(mmaps_generator_fast
  PRIVATE
    trinity-core-interface
    extractor_common
    common
    Recast
    Detour)

target_include_directories(mmaps_generator_fast
  PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR})

# the Generator sources are compiled in here, not imported from a library
target_compile_definitions(mmaps_generator_fast
  PRIVATE
    TRINITY_API_EXPORT_MMAPS_COMMON)

set_target_properties(mmaps_generator_fast
  PROPERTIES
    COMPILE_WARNING_AS_ERROR ${WITH_WARNINGS_AS_ERRORS}
    FOLDER "tools")

if(UNIX)
  install(TARGETS mmaps_generator_fast DESTINATION bin)
elseif(WIN32)
  install(TARGETS mmaps_generator_fast DESTINATION "${CMAKE_INSTALL_PREFIX}")
endif()
