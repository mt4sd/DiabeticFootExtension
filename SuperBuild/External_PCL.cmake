
set(proj PCL)

# Set dependency list
if (WIN32)
  set(${proj}_DEPENDS
    Eigen3
    Flann
  )
else() 
  set(${proj}_DEPENDS
  )
endif()

# Include dependent projects if any
ExternalProject_Include_Dependencies(${proj} PROJECT_VAR proj)

if(${SUPERBUILD_TOPLEVEL_PROJECT}_USE_SYSTEM_${proj})
  message(FATAL_ERROR "Enabling ${SUPERBUILD_TOPLEVEL_PROJECT}_USE_SYSTEM_${proj} is not supported !")
endif()

# Sanity checks
if(DEFINED PCL_DIR AND NOT EXISTS ${PCL_DIR})
  message(FATAL_ERROR "PCL_DIR [${PCL_DIR}] variable is defined but corresponds to nonexistent directory")
endif()

if(NOT DEFINED ${proj}_DIR AND NOT ${SUPERBUILD_TOPLEVEL_PROJECT}_USE_SYSTEM_${proj})

  ExternalProject_SetIfNotDefined(
   ${SUPERBUILD_TOPLEVEL_PROJECT}_${proj}_GIT_REPOSITORY
   "${EP_GIT_PROTOCOL}://github.com/PointCloudLibrary/pcl.git"
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

  set(pcl_common_c_flags "${ep_common_c_flags} -fPIC")

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
      -DCMAKE_C_FLAGS:STRING=${pcl_common_c_flags} 
      -DWITH_QT:BOOL=ON
      -DQt5_DIR:PATH=${Qt5_DIR}
      -DWITH_VTK:BOOL=ON
      -DVTK_DIR:PATH=${VTK_DIR}
      -DEIGEN_ROOT:PATH=${Eigen3_DIR}
      -DFLANN_ROOT:PATH=${Flann_DIR}
      -DPCL_SHARED_LIBS:BOOL=ON
      -DBUILD_apps:BOOL=OFF
      -DBUILD_examples:BOOL=OFF
      -DBUILD_outofcore:BOOL=OFF
      -DBUILD_people:BOOL=OFF
      -DBUILD_simulation:BOOL=OFF
      -DBUILD_tracking:BOOL=OFF
      -DBUILD_CUDA:BOOL=OFF
      -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
    DEPENDS
      ${${proj}_DEPENDS}
  )
  set(${proj}_DIR ${EP_INSTALL_DIR})
  
  if(UNIX)
    set(${proj}_DIR ${EP_INSTALL_DIR}/share/pcl-1.9/)
  endif()

  if (WIN32)
    set(${proj}_DIR ${EP_INSTALL_DIR}/cmake/)
  endif()

else()
  ExternalProject_Add_Empty(${proj} DEPENDS ${${proj}_DEPENDS})
endif()

mark_as_superbuild(${proj}_DIR:PATH)
ExternalProject_Message(${proj} "${proj}_DIR:${${proj}_DIR}")

