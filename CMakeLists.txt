cmake_minimum_required(VERSION 3.6)

set(PROJECT_NAME "KinectAvatar")
set(PROJECT_VERSION 0.0.1)

project(${PROJECT_NAME} VERSION ${PROJECT_VERSION} LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(SOURCES
    source/main.cpp 
)


add_executable(${PROJECT_NAME} ${SOURCES})

# ---- ugly hack ---- #
add_definitions(-D_ITERATOR_DEBUG_LEVEL=2)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# ------------ bgfx related stuff ------------- #
if( NOT BX_DIR )
	set( BX_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../bx" CACHE PATH "Location of bx." )
endif()
find_library(BX_LIB 
	NAMES bx bxDebug
)
if(NOT BX_LIB)
	message(FATAL_ERROR "bgfx not found.")
endif()
if( MSVC )
	target_include_directories( ${PROJECT_NAME} PUBLIC ${BX_DIR}/include/compat/msvc )
endif()

if( NOT BGFX_DIR )
	set( BGFX_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../bgfx" CACHE PATH "Location of bgfx." )
endif()
find_library(BGFX_LIB 
	NAMES bgfx bgfxDebug
)
if(NOT BGFX_LIB)
	message(FATAL_ERROR "bgfx not found.")
endif()

if( NOT BGFX_COMMON_DIR )
	set( BGFX_COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../bgfx/common" CACHE PATH "Location of bgfx common." )
endif()
find_library(BGFX_COMMON_LIB 
	NAMES example-common example-commonDebug
)
if(NOT BGFX_COMMON_LIB)
	message(FATAL_ERROR "bgfx-common not found.")
endif()


# ---------- ib-compress ------------#
file( GLOB IB_COMPRESS_SOURCES ${BGFX_DIR}/3rdparty/ib-compress/*.cpp ${BGFX_DIR}/3rdparty/ib-compress/*.h )
add_library( ib-compress STATIC ${IB_COMPRESS_SOURCES} )
target_include_directories( ib-compress PUBLIC ${BGFX_DIR}/3rdparty )
set_target_properties( ib-compress PROPERTIES FOLDER "bgfx/3rdparty" )


# ------------ imgui ---------------#
file( GLOB OCORNUT_IMGUI_SOURCES ${BGFX_DIR}/3rdparty/ocornut-imgui/*.cpp ${BGFX_DIR}/3rdparty/ocornut-imgui/*.h ${BGFX_DIR}/3rdparty/ocornut-imgui/*.inl )
add_library( ocornut-imgui STATIC EXCLUDE_FROM_ALL ${OCORNUT_IMGUI_SOURCES} )
target_compile_definitions( ocornut-imgui PRIVATE "-D_CRT_SECURE_NO_WARNINGS" "-D__STDC_FORMAT_MACROS" )
target_include_directories( ocornut-imgui PUBLIC ${BX_DIR}/include ${BGFX_DIR}/3rdparty )
if( MSVC )
	target_include_directories( ocornut-imgui PUBLIC ${BX_DIR}/include/compat/msvc )
endif()
target_link_libraries( ocornut-imgui PUBLIC ${BX_LIB} )
set_target_properties( ocornut-imgui PROPERTIES FOLDER "bgfx/3rdparty" )

# ------------ kinect 2 related stuff ------------- #
if( NOT KINECT2_DIR )
	set( KINECT2_DIR "${CMAKE_CURRENT_SOURCE_DIR}/kinect2" CACHE PATH "Location of kinect2." )
endif()

find_library(KINECT2_LIB 
	NAMES kinect20
)
if(NOT KINECT2_LIB)
	message(FATAL_ERROR "kinect2 library not found.")
endif()


# ------------ opencv related stuff ------------- #
find_package( OpenCV REQUIRED )


# ------------- eigen + SuiteSparse ---------------------- #
set(CLAPACK_BUILD_PATH ${PROJECT_SOURCE_DIR}/../repositories/clapack-3.2.1-CMAKE/build/)
find_library(CLAPACK_COMMON_LIBRARY
	NAMES libf2cd
	HINTS ${CLAPACK_BUILD_PATH}/F2CLIBS/libf2c/Debug)
find_library(BLAS_LIBRARY 
	NAMES blasd
	HINTS ${CLAPACK_BUILD_PATH}/BLAS/SRC/Debug/)
find_library(LAPACK_LIBRARY
	NAMES lapackd
	HINTS ${CLAPACK_BUILD_PATH}/SRC/Debug/)

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${PROJECT_SOURCE_DIR}/../repositories/suitesparse-metis-for-windows/SuiteSparse/SuiteSparse_config)

find_package(Cholmod REQUIRED)
if(CHOLMOD_FOUND)
  add_definitions("-DEIGEN_CHOLMOD_SUPPORT")
  include_directories(${CHOLMOD_INCLUDES})
  set(SPARSE_LIBS ${SPARSE_LIBS} ${CHOLMOD_LIBRARIES} ${CLAPACK_COMMON_LIBRARY}  ${BLAS_LIBRARY} ${LAPACK_LIBRARY}  )
  set(CHOLMOD_ALL_LIBS  ${CHOLMOD_LIBRARIES} ${BLAS_LIBRARY} ${LAPACK_LIBRARY})
endif()

find_package(SPQR REQUIRED)
if(SPQR_FOUND)
	add_definitions("-DEIGEN_SPQR_SUPPORT")
	include_directories(${SPQR_INCLUDES})
	set(SPQR_ALL_LIBS ${SPQR_LIBRARIES} ${CHOLMOD_LIBRARIES} ${CLAPACK_COMMON_LIBRARY} ${BLAS_LIBRARY} ${LAPACK_LIBRARIES}  )
	set(SPARSE_LIBS ${SPARSE_LIBS} ${SPQR_ALL_LIBS})
endif()

# ------------------- PCL ---------------------- #
find_package(PCL 1.8 REQUIRED COMPONENTS common features io registration surface visualization segmentation)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ----------------- opengm --------------------- #
if( NOT opengm_DIR )
	set( opengm_DIR "${CMAKE_CURRENT_SOURCE_DIR}/opengm" CACHE PATH "Location of opengm.")
endif()
target_include_directories(${PROJECT_NAME} PUBLIC ${opengm_DIR}/include)

# ------------ put it all together ------------- #
target_include_directories(${PROJECT_NAME}
    PUBLIC ${BX_DIR}/include ${BGFX_DIR}/include ${BGFX_COMMON_DIR} ${KINECT2_DIR}/inc ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC ${BX_LIB} ${BGFX_LIB} ${BGFX_COMMON_LIB} ${KINECT2_LIB} ib-compress ocornut-imgui ${OpenCV_LIBS} ${PCL_LIBRARIES} ${SPARSE_LIBS}
)