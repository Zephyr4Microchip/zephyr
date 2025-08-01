# Copyright (c) 2025 IAR Systems AB
#
# SPDX-License-Identifier: Apache-2.0

set_property(TARGET linker PROPERTY devices_start_symbol "_device_list_start")
find_program(CMAKE_LINKER
  NAMES ${CROSS_COMPILE}${IAR_LINKER}
  PATHS ${TOOLCHAIN_HOME}
  PATH_SUFFIXES bin
  NO_DEFAULT_PATH
)

if(CONFIG_IAR_DATA_INIT)
  zephyr_linker_section(NAME .iar.init_table KVMA RAM_REGION GROUP RODATA_REGION)
endif()

add_custom_target(${IAR_LINKER})
set(ILINK_THUMB_CALLS_WARNING_SUPPRESSED)
set(IAR_LIB_USED)
function(toolchain_ld_force_undefined_symbols "")
#  foreach(symbol ${ARGN})
#    zephyr_link_libraries(--place_holder=${symbol})
#  endforeach()
endfunction()

# NOTE: ${linker_script_gen} will be produced at build-time; not at configure-time
macro(configure_linker_script linker_script_gen linker_pass_define)
  set(extra_dependencies ${ARGN})
  set(STEERING_FILE "${CMAKE_CURRENT_BINARY_DIR}/${linker_script_gen}.xcl")
  set(STEERING_FILE_ARG "-DSTEERING_FILE=${STEERING_FILE}")
  set(cmake_linker_script_settings
      ${PROJECT_BINARY_DIR}/include/generated/ld_script_settings_${linker_pass_define}.cmake
  )
  if("${linker_pass_define}" STREQUAL "LINKER_ZEPHYR_PREBUILT")
    set(ILINK_THUMB_CALLS_WARNING_SUPPRESSED "--diag_suppress=Lt056")
  else()
    set(ILINK_THUMB_CALLS_WARNING_SUPPRESSED "")
  endif()
  if(CONFIG_IAR_LIBC OR CONFIG_IAR_LIBCPP)
    set(IAR_LIB_USED "-DIAR_LIBC=1")
  else()
    set(IAR_LIB_USED "")
  endif()
  zephyr_linker_generate_linker_settings_file(${cmake_linker_script_settings})

  add_custom_command(
    OUTPUT ${linker_script_gen}
           ${STEERING_FILE}
    DEPENDS
           ${extra_dependencies}
           ${cmake_linker_script_settings}
           ${DEVICE_API_LD_TARGET}
    COMMAND ${CMAKE_COMMAND}
      -C ${cmake_linker_script_settings}
      -DPASS="${linker_pass_define}"
      ${STEERING_FILE_ARG}
      -DOUT_FILE=${CMAKE_CURRENT_BINARY_DIR}/${linker_script_gen}
      ${IAR_LIB_USED}
      -P ${ZEPHYR_BASE}/cmake/linker/iar/config_file_script.cmake
  )

endmacro()

function(toolchain_ld_link_elf)
  cmake_parse_arguments(
    TOOLCHAIN_LD_LINK_ELF                                     # prefix of output variables
    ""                                                        # list of names of the boolean arguments
    "TARGET_ELF;OUTPUT_MAP;LINKER_SCRIPT"                     # list of names of scalar arguments
    "LIBRARIES_PRE_SCRIPT;LIBRARIES_POST_SCRIPT;DEPENDENCIES" # list of names of list arguments
    ${ARGN}                                                   # input args to parse
  )

  set(whole_libs)
  foreach(lib ${WHOLE_ARCHIVE_LIBS})
	  list(APPEND whole_libs --whole_archive ${lib})
  endforeach()

  set(ILINK_SEMIHOSTING)
  set(ILINK_BUFFERED_WRITE)
  if(${CONFIG_IAR_SEMIHOSTING})
    set(ILINK_SEMIHOSTING "--semihosting")
  endif()
  if(${CONFIG_IAR_BUFFERED_WRITE})
    set(ILINK_BUFFERED_WRITE "--redirect __write=__write_buffered")
  endif()

  set(ILINK_XCL "-f ${TOOLCHAIN_LD_LINK_ELF_LINKER_SCRIPT}.xcl")

  set(ILINK_TLS_LIBRARY)
  if(${CONFIG_THREAD_LOCAL_STORAGE})
    set(ILINK_TLS_LIBRARY "--threaded_lib=manual")
  endif()

  set(ILINK_TZONE_LIBRARY)
  # if(${CONFIG_IAR_LIBC})
  #   set(ILINK_TZONE_LIBRARY "--timezone_lib")
  # endif()

  target_link_libraries(
    ${TOOLCHAIN_LD_LINK_ELF_TARGET_ELF}
    ${TOOLCHAIN_LD_LINK_ELF_LIBRARIES_PRE_SCRIPT}
    --config=${TOOLCHAIN_LD_LINK_ELF_LINKER_SCRIPT}
    ${TOOLCHAIN_LD_LINK_ELF_LIBRARIES_POST_SCRIPT}
    --map=${TOOLCHAIN_LD_LINK_ELF_OUTPUT_MAP}
    --log_file=${TOOLCHAIN_LD_LINK_ELF_OUTPUT_MAP}.log

    ${whole_libs}
    ${NO_WHOLE_ARCHIVE_LIBS}
    $<TARGET_OBJECTS:${OFFSETS_LIB}>
    --entry=$<TARGET_PROPERTY:linker,ENTRY>

    ${ILINK_SEMIHOSTING}
    ${ILINK_BUFFERED_WRITE}
    ${ILINK_TLS_LIBRARY}
    ${ILINK_TZONE_LIBRARY}
    ${ILINK_THUMB_CALLS_WARNING_SUPPRESSED}
    # Do not remove symbols
    #--no_remove
    ${ILINK_XCL}

    ${TOOLCHAIN_LIBS_OBJECTS}

    ${TOOLCHAIN_LD_LINK_ELF_DEPENDENCIES}
  )
endfunction(toolchain_ld_link_elf)

include(${ZEPHYR_BASE}/cmake/linker/ld/target_relocation.cmake)
include(${ZEPHYR_BASE}/cmake/linker/ld/target_configure.cmake)
