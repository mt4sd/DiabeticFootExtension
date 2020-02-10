
set(proj Eigen3)

# Set dependency list
set(${proj}_DEPENDS
  )

# Include dependent projects if any
ExternalProject_Include_Dependencies(${proj} PROJECT_VAR proj)

if(${SUPERBUILD_TOPLEVEL_PROJECT}_USE_SYSTEM_${proj})
  message(FATAL_ERROR "Enabling ${SUPERBUILD_TOPLEVEL_PROJECT}_USE_SYSTEM_${proj} is not supported !")
endif()

# Sanity checks
if(DEFINED Eigen3_DIR AND NOT EXISTS ${Eigen3_DIR})
  message(FATAL_ERROR "Eigen3_DIR [${Eigen3_DIR}] variable is defined but corresponds to nonexistent directory")
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
    URL "https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip"
    SOURCE_DIR ${EP_SOURCE_DIR}
    CONFIGURE_COMMAND ${CMAKE_COMMAND} -E echo ""
    BUILD_COMMAND ${CMAKE_COMMAND} -E echo ""
    INSTALL_COMMAND "" 
    DEPENDS
      ${${proj}_DEPENDS}
    )
  set(${proj}_DIR ${EP_SOURCE_DIR})
  # set(EIGEN_INCLUDE_DIR ${proj}_DIR)

else()
  ExternalProject_Add_Empty(${proj} DEPENDS ${${proj}_DEPENDS})
endif()

mark_as_superbuild(${proj}_DIR:PATH)
ExternalProject_Message(${proj} "${proj}_DIR:${${proj}_DIR}")
# mark_as_superbuild(EIGEN_INCLUDE_DIR:PATH)
# ExternalProject_Message(${proj} "EIGEN_INCLUDE_DIR:${EIGEN_INCLUDE_DIR}")

