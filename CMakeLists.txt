cmake_minimum_required(VERSION 3.3)

project(voxl-app-manager)

# James' standard list of cmake flags
set(CMAKE_C_FLAGS "-g -std=gnu99 -Wall -Wextra -Wuninitialized \
	-Wunused-variable -Wdouble-promotion -Wmissing-prototypes \
	-Wmissing-declarations -Werror=undef -Wno-unused-function ${CMAKE_C_FLAGS}")



### INSTALL SHARED LIBRARY FOR SAMPLE APP
# set(LIBNAME voxl_app_template)
set(LIBNAME voxl_app_figure_eight)

set(LIB_INSTALL_DIR /etc/voxl-apps/)

# Build lib from all source files
file(GLOB shared_lib_src_files app_example/*.c)
add_library(${LIBNAME} SHARED ${shared_lib_src_files})
target_link_libraries(${LIBNAME} LINK_PUBLIC m)

# Set the library prefix to empty string (by default CMake will prepend "lib")
set_target_properties(${LIBNAME} PROPERTIES PREFIX "")

# make the include directory public for install
file(GLOB LIB_HEADERS include/*.h)
set_target_properties(${LIBNAME} PROPERTIES PUBLIC_HEADER "${LIB_HEADERS}")

# make sure everything is installed where we want
install(
	TARGETS			${LIBNAME}
	LIBRARY			DESTINATION ${LIB_INSTALL_DIR}
	RUNTIME			DESTINATION /usr/bin
	PUBLIC_HEADER	DESTINATION /usr/include
)



### INSTALL VOXL-APP-MANAGER SERVICE
set(TARGET voxl-app-manager)

# for VOXL, install 64-bit libraries to lib64, 32-bit libs go in /usr/lib
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch64")
    set(APP_MANAGER_INSTALL_DIR /usr/lib64)
else()
    set(APP_MANAGER_INSTALL_DIR /usr/lib)
endif()

# Build from all source files
file(GLOB all_src_files src/*.c)
add_executable(${TARGET} ${all_src_files})
include_directories(include)

find_library(MODAL_JSON     modal_json     HINTS  /usr/lib /usr/lib64)
find_library(MODAL_PIPE     modal_pipe     HINTS  /usr/lib /usr/lib64)
find_library(VOXL_CUTILS    voxl_cutils    HINTS  /usr/lib /usr/lib64)
find_library(MODAL_JOURNAL  modal_journal  HINTS  /usr/lib /usr/lib64)
find_library(RC_MATH        rc_math        HINTS  /usr/lib /usr/lib64)

target_link_libraries(${TARGET} pthread m ${MODAL_JSON} ${MODAL_PIPE} ${VOXL_CUTILS} ${MODAL_JOURNAL} ${RC_MATH} ${CMAKE_DL_LIBS})

# make sure everything is installed where we want
install(
	TARGETS			${TARGET}
	LIBRARY			DESTINATION ${APP_MANAGER_INSTALL_DIR}
	RUNTIME			DESTINATION /usr/bin
	PUBLIC_HEADER	DESTINATION /usr/include
)




### INSTALL VOXL-INSPECT APPS UTILITY
set(TARGET voxl-inspect-apps)

# for VOXL, install 64-bit libraries to lib64, 32-bit libs go in /usr/lib
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch64")
    set(INSPECT_APPS_INSTALL_DIR /usr/lib64)
else()
    set(INSPECT_APPS_INSTALL_DIR /usr/lib)
endif()

# Build from all source files
file(GLOB inspect_apps inspect_apps/*.c)
add_executable(${TARGET} ${inspect_apps})
include_directories(include)

find_library(MODAL_JSON  modal_json  HINTS /usr/lib /usr/lib64)
find_library(MODAL_PIPE  modal_pipe  HINTS /usr/lib /usr/lib64)
find_library(VOXL_CUTILS voxl_cutils HINTS /usr/lib /usr/lib64)

target_link_libraries(${TARGET} pthread ${MODAL_JSON} ${MODAL_PIPE} ${VOXL_CUTILS} ${CMAKE_DL_LIBS})

# make sure everything is installed where we want
install(
	TARGETS			${TARGET}
	LIBRARY			DESTINATION ${INSPECT_APPS_INSTALL_DIR}
	RUNTIME			DESTINATION /usr/bin
	PUBLIC_HEADER	DESTINATION /usr/include
)
