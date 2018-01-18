cmake_minimum_required(VERSION 2.8)

set( CMAKE_EXPORT_COMPILE_COMMANDS 1)
# add prefix to every string in a variable
FUNCTION(PREPEND var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND)

FUNCTION(PREPEND_CPP var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.cpp")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_CPP)

FUNCTION(PREPEND_H var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.h")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_H)

project(aris)

# set the platform.h, which define the OS platform
if(UNIX)
	set(PLATFORM "UNIX")
	set(CMAKE_C_COMPILER "/usr/bin/gcc-4.8")
	set(CMAKE_CXX_COMPILER "/usr/bin/g++-4.8")
    #set(CMAKE_C_COMPILER "/usr/bin/clang")
    #set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
endif(UNIX)
if(WIN32)
	set(PLATFORM "PLATFORM_IS_WINDOWS")
endif(WIN32)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_BUILD_TYPE "Release")

# set compile options
if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
endif(UNIX)

# set definitions to disable some warning
if(WIN32)
add_definitions(-DWIN32 -D_WINSOCK_DEPRECATED_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS)
endif(WIN32)
if(UNIX)
add_definitions(-DUNIX)
endif(UNIX)

# set debug and release options
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CONFIGURATION_TYPES Debug Release)

# set include and lib folders
if(UNIX)
set(GENERAL_INCL_DIR "/usr/Aris_Dependent")
set(SYSTEM_LINK_LIB pthread)
set(XENOMAI_INCL_DIR "/usr/Aris_Dependent/xenomai/include")
set(XENOMAI_LINK_DIR "/usr/Aris_Dependent/xenomai/lib")
set(XENOMAI_LINK_LIB native rtdm xenomai)
set(ETHERLAB_INCL_DIR "/usr/Aris_Dependent/etherlab/include")
set(ETHERLAB_LINK_DIR "/usr/Aris_Dependent/opt/etherlab/lib")
set(ETHERLAB_LINK_LIB ethercat_rtdm rt)
set(ARIS_INCL_DIR ${PROJECT_BINARY_DIR} src/aris_server src/aris_core  src/aris_control src/aris_model)
set(ARIS_LINK_LIB aris_server aris_dynamic aris_control aris_core aris_model)
set(EIGEN_INCL_DIR "/usr/Aris_Dependent/Eigen")

set(ALL_INCL_DIR ${ARIS_INCL_DIR} ${ETHERLAB_INCL_DIR} ${XENOMAI_INCL_DIR} ${GENERAL_INCL_DIR} ${EIGEN_INCL_DIR})
set(ALL_LINK_DIR ${ARIS_LINK_DIR} ${ETHERLAB_LINK_DIR} ${XENOMAI_LINK_DIR})
set(ALL_LINK_LIB ${ARIS_LINK_LIB} ${ETHERLAB_LINK_LIB} ${XENOMAI_LINK_LIB} ${SYSTEM_LINK_LIB})
endif(UNIX)

include_directories(${ALL_INCL_DIR})
link_directories(${ALL_LINK_DIR})


# aris core project
set(SOURCE aris_core_msg aris_core_socket aris_core_expression_calculator aris_core_msg_loop aris_core_xml tinyxml2)
PREPEND_CPP(FULL_SRC src/aris_core ${SOURCE})
PREPEND_H(FULL_H src/aris_core ${SOURCE})
add_library(aris_core SHARED ${FULL_SRC} ${FULL_H} src/aris_core/aris_core.h)
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H} src/aris_core/aris_core.h)

# aris control project
set(SOURCE aris_control_motion aris_control_ethercat aris_control_pipe MadgwickAHRS)
PREPEND_CPP(FULL_SRC src/aris_control ${SOURCE})
PREPEND_H(FULL_H src/aris_control ${SOURCE})
add_library(aris_control SHARED ${FULL_SRC} ${FULL_H} src/aris_control/aris_control.h)
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H} src/aris_control/aris_control.h)

# aris control server project
set(SOURCE aris_server aris_node)
PREPEND_CPP(FULL_SRC src/aris_server ${SOURCE})
PREPEND_H(FULL_H src/aris_server ${SOURCE})
add_library(aris_server SHARED ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})

# aris model project
set(SOURCE aris_model)
PREPEND_CPP(FULL_SRC src/aris_model ${SOURCE})
PREPEND_H(FULL_H src/aris_model ${SOURCE})
add_library(aris_model SHARED ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})
############################################ install aris ############################################
if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/aris")
endif(UNIX)

install(DIRECTORY sh DESTINATION .)
install(TARGETS aris_core aris_control aris_server aris_model ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(FILES ${INCLUDE_HEADER_FILES} src/aris.h DESTINATION include)
install(DIRECTORY src/aris_control/resource/ DESTINATION resource/aris_control)

############################################ uninstall aris ############################################
if(UNIX)
	add_custom_target(uninstall rm -r /usr/aris)
endif(UNIX)

