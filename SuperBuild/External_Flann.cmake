
set(proj Flann)

# Set dependency list
set(${proj}_DEPENDS
  )

# Include dependent projects if any
ExternalProject_Include_Dependencies(${proj} PROJECT_VAR proj)

if(${SUPERBUILD_TOPLEVEL_PROJECT}_USE_SYSTEM_${proj})
  message(FATAL_ERROR "Enabling ${SUPERBUILD_TOPLEVEL_PROJECT}_USE_SYSTEM_${proj} is not supported !")
endif()

# Sanity checks
if(DEFINED Flann_DIR AND NOT EXISTS ${Flann_DIR})
  message(FATAL_ERROR "Flann_DIR [${Flann_DIR}] variable is defined but corresponds to nonexistent directory")
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

  set(EP_SOURCE_DIR ${CMAKE_BINARY_DIR}/${proj})

  ExternalProject_Add(${proj}
    ${${proj}_EP_ARGS}
    URL "https://github.com//mariusmuja/flann/archive/1.9.1.zip"
    SOURCE_DIR ${EP_SOURCE_DIR}
    CONFIGURE_COMMAND ${CMAKE_COMMAND} -E echo ""
    BUILD_COMMAND ${CMAKE_COMMAND} -E echo ""
    INSTALL_COMMAND "" 
    DEPENDS
      ${${proj}_DEPENDS}
    )
  set(${proj}_DIR ${EP_SOURCE_DIR})
  if(UNIX)
    set(${proj}_DIR ${EP_SOURCE_DIR}/share/cmake/Torch/)
  endif()

else()
  ExternalProject_Add_Empty(${proj} DEPENDS ${${proj}_DEPENDS})
endif()

mark_as_superbuild(${proj}_DIR:PATH)
ExternalProject_Message(${proj} "${proj}_DIR:${${proj}_DIR}")

