
set(proj Torch)

# Set dependency list
set(${proj}_DEPENDS
  )

# Include dependent projects if any
ExternalProject_Include_Dependencies(${proj} PROJECT_VAR proj)

if(${SUPERBUILD_TOPLEVEL_PROJECT}_USE_SYSTEM_${proj})
  message(FATAL_ERROR "Enabling ${SUPERBUILD_TOPLEVEL_PROJECT}_USE_SYSTEM_${proj} is not supported !")
endif()

# Sanity checks
if(DEFINED Torch_DIR AND NOT EXISTS ${Torch_DIR})
  message(FATAL_ERROR "Torch_DIR [${Torch_DIR}] variable is defined but corresponds to nonexistent directory")
endif()

if(NOT DEFINED ${proj}_DIR AND NOT ${SUPERBUILD_TOPLEVEL_PROJECT}_USE_SYSTEM_${proj})
 
  ExternalProject_SetIfNotDefined(
   ${SUPERBUILD_TOPLEVEL_PROJECT}_${proj}_GIT_REPOSITORY
   QUIET
   )

  ExternalProject_SetIfNotDefined(
   ${SUPERBUILD_TOPLEVEL_PROJECT}_${proj}_GIT_TAG
   QUIET
   )

  if (UNIX)
    set(${proj}_URL https://download.pytorch.org/libtorch/cu101/libtorch-cxx11-abi-shared-with-deps-1.3.1.zip)
  else()
    # set(${proj}_URL https://download.pytorch.org/libtorch/cpu/libtorch-win-shared-with-deps-1.4.0.zip) #CPU, not working
    set(${proj}_URL https://download.pytorch.org/libtorch/cu101/libtorch-win-shared-with-deps-1.4.0.zip)
  endif()

  set(EP_SOURCE_DIR ${CMAKE_BINARY_DIR}/${proj})

  ExternalProject_Add(${proj}
    ${${proj}_EP_ARGS}
    URL ${${proj}_URL}
    SOURCE_DIR ${EP_SOURCE_DIR}
    CONFIGURE_COMMAND ${CMAKE_COMMAND} -E echo
      "This CONFIGURE_COMMAND is just here as a placeholder."
      "Remove this line to enable configuring of a real CMake based external project"
    BUILD_COMMAND ${CMAKE_COMMAND} -E echo
      "This BUILD_COMMAND is just here as a placeholder."
      "Remove this line to enable building of a real CMake based external project"
    INSTALL_COMMAND "" 
    DEPENDS
      ${${proj}_DEPENDS}
    )
  
  set(${proj}_DIR ${EP_SOURCE_DIR}/share/cmake/Torch/)
  

else()
  ExternalProject_Add_Empty(${proj} DEPENDS ${${proj}_DEPENDS})
endif()

mark_as_superbuild(${proj}_DIR:PATH)
ExternalProject_Message(${proj} "${proj}_DIR:${${proj}_DIR}")

