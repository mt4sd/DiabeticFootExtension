
set(proj Foo)

# Set dependency list
set(${proj}_DEPENDS
  )

# Include dependent projects if any
ExternalProject_Include_Dependencies(${proj} PROJECT_VAR proj)

if(${SUPERBUILD_TOPLEVEL_PROJECT}_USE_SYSTEM_${proj})
  message(FATAL_ERROR "Enabling ${SUPERBUILD_TOPLEVEL_PROJECT}_USE_SYSTEM_${proj} is not supported !")
endif()

# Sanity checks
if(DEFINED Foo_DIR AND NOT EXISTS ${Foo_DIR})
  message(FATAL_ERROR "Foo_DIR [${Foo_DIR}] variable is defined but corresponds to nonexistent directory")
endif()

if(NOT DEFINED ${proj}_DIR AND NOT ${SUPERBUILD_TOPLEVEL_PROJECT}_USE_SYSTEM_${proj})

  ExternalProject_SetIfNotDefined(
   ${SUPERBUILD_TOPLEVEL_PROJECT}_${proj}_GIT_REPOSITORY
  #  "${EP_GIT_PROTOCOL}://github.com/PointCloudLibrary/pcl.git"
   "https://github.com/PointCloudLibrary/pcl.git"
   QUIET
   )

  ExternalProject_SetIfNotDefined(
   ${SUPERBUILD_TOPLEVEL_PROJECT}_${proj}_GIT_TAG
   "pcl-1.9.1"
   QUIET
   )

  set(EP_SOURCE_DIR ${CMAKE_BINARY_DIR}/${proj})
  set(EP_BINARY_DIR ${CMAKE_BINARY_DIR}/${proj}-build)
  set(EP_INSTALL_DIR ${CMAKE_BINARY_DIR}/${proj}-install)

  ExternalProject_Add(${proj}
    ${${proj}_EP_ARGS}
    GIT_REPOSITORY "${${SUPERBUILD_TOPLEVEL_PROJECT}_${proj}_GIT_REPOSITORY}"
    GIT_TAG "${${SUPERBUILD_TOPLEVEL_PROJECT}_${proj}_GIT_TAG}"
    # DOWNLOAD_COMMAND ${CMAKE_COMMAND} -E echo "Remove this line and uncomment GIT_REPOSITORY and GIT_TAG"
    SOURCE_DIR ${EP_SOURCE_DIR}
    BINARY_DIR ${EP_BINARY_DIR}
    INSTALL_DIR ${EP_INSTALL_DIR}
    CMAKE_CACHE_ARGS
      -DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
      -DCMAKE_CXX_FLAGS:STRING=${ep_common_cxx_flags}
      -DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
      -DCMAKE_C_FLAGS:STRING=${ep_common_c_flags}
      -DWITH_QT:BOOL=ON
      -DQt5_DIR:PATH=${Qt5_DIR}
      -DWITH_VTK:BOOL=ON
      -DVTK_DIR:PATH=${VTK_DIR}
      -DPCL_SHARED_LIBS:BOOL=ON
      -DBUILD_apps:BOOL=OFF
      -DBUILD_examples:BOOL=OFF
      -DBUILD_outofcore:BOOL=OFF
      -DBUILD_people:BOOL=OFF
      -DBUILD_simulation:BOOL=OFF
      -DBUILD_tracking:BOOL=OFF
      -DBUILD_CUDA:BOOL=OFF
      -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>

      # # Compiler settings
      # -DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
      # -DCMAKE_C_FLAGS:STRING=${ep_common_c_flags}
      # -DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
      # -DCMAKE_CXX_FLAGS:STRING=${ep_common_cxx_flags}
      # -DCMAKE_CXX_STANDARD:STRING=${CMAKE_CXX_STANDARD}
      # -DCMAKE_CXX_STANDARD_REQUIRED:BOOL=${CMAKE_CXX_STANDARD_REQUIRED}
      # -DCMAKE_CXX_EXTENSIONS:BOOL=${CMAKE_CXX_EXTENSIONS}
      # # Output directories
      # -DCMAKE_RUNTIME_OUTPUT_DIRECTORY:PATH=${CMAKE_BINARY_DIR}/${Slicer_THIRDPARTY_BIN_DIR}
      # -DCMAKE_LIBRARY_OUTPUT_DIRECTORY:PATH=${CMAKE_BINARY_DIR}/${Slicer_THIRDPARTY_LIB_DIR}
      # -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY:PATH=${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
      # # Install directories
      # # XXX The following two variables should be updated to match the
      # #     requirements of a real CMake based external project
      # # XXX Then, this comment and the one above should be removed. Really.
      # -DFOO_INSTALL_RUNTIME_DIR:STRING=${Slicer_INSTALL_THIRDPARTY_LIB_DIR}
      # -DFOO_INSTALL_LIBRARY_DIR:STRING=${Slicer_INSTALL_THIRDPARTY_LIB_DIR}
      # # Output directories for CLIs
      # #-DSlicerExecutionModel_DEFAULT_CLI_RUNTIME_OUTPUT_DIRECTORY:PATH=${SlicerExecutionModel_DEFAULT_CLI_RUNTIME_OUTPUT_DIRECTORY}
      # #-DSlicerExecutionModel_DEFAULT_CLI_RUNTIME_LIBRARY_DIRECTORY:PATH=${SlicerExecutionModel_DEFAULT_CLI_LIBRARY_OUTPUT_DIRECTORY}
      # #-DSlicerExecutionModel_DEFAULT_CLI_RUNTIME_ARCHIVE_DIRECTORY:PATH=${SlicerExecutionModel_DEFAULT_CLI_ARCHIVE_OUTPUT_DIRECTORY}
      # # Options
      # -DBUILD_TESTING:BOOL=OFF
      # # Dependencies
      # # -DBar_DIR:PATH=${Bar_DIR}
    # CONFIGURE_COMMAND ${CMAKE_COMMAND} -E echo
    #   "This CONFIGURE_COMMAND is just here as a placeholder."
    #   "Remove this line to enable configuring of a real CMake based external project"
    # BUILD_COMMAND ${CMAKE_COMMAND} -E echo
    #   "This BUILD_COMMAND is just here as a placeholder."
    #   "Remove this line to enable building of a real CMake based external project"
    # INSTALL_COMMAND ""
    # DEPENDS
    #   ${${proj}_DEPENDS}
    )
  set(${proj}_DIR ${EP_BINARY_DIR})

else()
  ExternalProject_Add_Empty(${proj} DEPENDS ${${proj}_DEPENDS})
endif()

mark_as_superbuild(${proj}_DIR:PATH)
