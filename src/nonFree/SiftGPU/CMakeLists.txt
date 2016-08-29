cmake_minimum_required(VERSION 3.0.2)
project(SiftGPU)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/src/cmake/")
add_definitions("-DWINDOW_PREFER_GLUT")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wall -Wno-deprecated -pthread ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall -Wno-deprecated -pthread ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 ")


#if (APPLE)
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=core2 -mfpmath=sse ")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=core2 -mfpmath=sse ")
#endif ()
find_package(DevIL REQUIRED)
include_directories(${IL_INCLUDE_DIR})
link_libraries(${IL_LIBRARIES})
#link_libraries(IL_LIBRARIES)
if(NOT IL_FOUND)
    message(ERROR "IL not found")
endif(NOT IL_FOUND)

#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
link_libraries(${GLUT_LIBRARIES})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
link_libraries(${OPENGL_LIBRARIES})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)


#########################################################
# FIND OPENCL
#########################################################
find_package(OpenCL)
if (OpenCL_FOUND)
    include_directories(${OpenCL_INCLUDE_DIRS})
    link_directories(${OpenCL_LIBRARIES})
    add_definitions("-DCL_SIFTGPU_ENABLED")
        link_libraries(${OpenCL_LIBRARIES})
    set(OPENCL_PROGRAM_SOURCE
            src/SiftGPU/ProgramCL.cpp
            src/SiftGPU/ProgramCL.h
            src/SiftGPU/PyramidCL.cpp
            src/SiftGPU/PyramidCL.h
            src/SiftGPU/CLTexImage.cpp
            src/SiftGPU/CLTexImage.h)
ELSE()
    MESSAGE("OpenCL Not Found, disbaled  by default")
endif ()


#########################################################
# FIND GLEW
#########################################################
find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})
link_libraries(${GLEW_LIBRARIES})
if(NOT GLEW_FOUND)
    message("GLEW IS NOT FOUND!")
endif()

#########################################################
# FIND CUDA
#########################################################
find_package(CUDA)
if(CUDA_FOUND)

    include_directories(${CUDA_INCLUDE_DIRS})
        link_directories()
    add_definitions("-DCUDA_SIFTGPU_ENABLED")
        set(ENABLE_CUDA TRUE)
    SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS};-D_FORCE_INLINES;-O2;-Xcompiler;-fPIC;-DCUDA_SIFTGPU_ENABLED")
    CUDA_INCLUDE_DIRECTORIES($(CMAKE_CURRENT_SOURCE_DIR)/include)
#    link_directories(${})
    set(CUDA_PROGRAM_SOURCE
            src/SiftGPU/CuTexImage.cpp
            src/SiftGPU/CuTexImage.h
            src/SiftGPU/PyramidCU.cpp
            src/SiftGPU/PyramidCU.h
            src/SiftGPU/ProgramCU.h
            src/SiftGPU/SiftMatchCU.cpp
            src/SiftGPU/SiftMatchCU.h)
ELSE()
    message("CUDA IS NOT FOUND. IF YOUR CUDA TOOLKIT IS INSTALLED, PLZ SET CUDA_TOOLKIT_ROOT_DIR AND TRY ONCE AGAIN.")
endif()

set(SOURCE_FILES
        include/GL/glew.h
        include/GL/glut.h
        include/GL/wglew.h
        include/IL/il.h
        src/SiftGPU/FrameBufferObject.cpp
        src/SiftGPU/FrameBufferObject.h
        src/SiftGPU/GlobalUtil.cpp
        src/SiftGPU/GlobalUtil.h
        src/SiftGPU/GLTexImage.cpp
        src/SiftGPU/GLTexImage.h
        src/SiftGPU/LiteWindow.h
        src/SiftGPU/ProgramCG.cpp
        src/SiftGPU/ProgramCG.h
        src/SiftGPU/ProgramGLSL.cpp
        src/SiftGPU/ProgramGLSL.h
        src/SiftGPU/ProgramGPU.cpp
        src/SiftGPU/ProgramGPU.h
        src/SiftGPU/PyramidGL.cpp
        src/SiftGPU/PyramidGL.h
        src/SiftGPU/ShaderMan.cpp
        src/SiftGPU/ShaderMan.h
        src/SiftGPU/SiftGPU.cpp
        src/SiftGPU/SiftGPU.h
        src/SiftGPU/SiftMatch.cpp
        src/SiftGPU/SiftMatch.h
        src/SiftGPU/SiftPyramid.cpp
        src/SiftGPU/SiftPyramid.h
        src/SiftGPU/SiftFileLoader.cpp
        src/SiftGPU/SiftFileLoader.h
        ${OPENCL_PROGRAM_SOURCE})


#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")


if(CUDA_FOUND)
    set(CUDA_PROGRAM_SOURCE
            ${CUDA_PROGRAM_SOURCE}
            src/SiftGPU/ProgramCU.cu)
    set(SOURCE_FILES ${SOURCE_FILES}
            ${CUDA_PROGRAM_SOURCE}
           src/SiftGPU/ )
    CUDA_ADD_LIBRARY(SiftGPUStatic STATIC ${SOURCE_FILES})
    CUDA_ADD_LIBRARY(SiftGPU SHARED ${SOURCE_FILES} )
    target_link_libraries(SiftGPU ${CUDA_LIBRARIES})
    target_link_libraries(SiftGPUStatic ${CUDA_LIBRARIES})
else()
    add_library(SiftGPUStatic STATIC ${SOURCE_FILES} )
    add_library(SiftGPU SHARED ${SOURCE_FILES} )
ENDIF()

INSTALL(TARGETS SiftGPUStatic DESTINATION lib EXPORT openMVG-targets)
SET_PROPERTY(TARGET SiftGPUStatic PROPERTY FOLDER OpenMVG/nonFree)
INSTALL(TARGETS SiftGPU DESTINATION lib EXPORT openMVG-targets)
SET_PROPERTY(TARGET SiftGPU PROPERTY FOLDER OpenMVG/nonFree)

INSTALL(
        DIRECTORY .
        DESTINATION include/openMVG_dependencies/nonFree/SiftGPU
        COMPONENT headers
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)


#
#set(TESTWINGLUT_SOURCE
#        src/TestWin/BasicTestWin.cpp
#        src/TestWin/BasicTestWin.h
#        src/TestWin/TestWinGlut.cpp
#        src/TestWin/TestWinGlut.h
#        )
#add_executable(TestWinGlut ${TESTWINGLUT_SOURCE})
#
#target_link_libraries(TestWinGlut SiftGPU)
#if(ENABLE_CUDA)
#    target_link_libraries(TestWinGlut ${CUDA_LIBRARIES})
#ENDIF()
#
#set(SIMPLESIFT_SOURCE src/TestWin/SimpleSIFT.cpp)
#add_executable(SimpleSIFT ${SIMPLESIFT_SOURCE})
#target_link_libraries(SimpleSIFT SiftGPU dl)
#
#
#SET(SIFT_INC_DIR src/SiftGPU)
#include_directories(${SIFT_INC_DIR})
#set(SIFT_CONSOLE_SOURCE src/TestWin/SiftConsole.cpp src/TestWin/SiftConsole.h)
#add_executable(SiftConsole ${SIFT_CONSOLE_SOURCE})
#target_link_libraries(SiftConsole SiftGPU)
#
#set(MATCH_CONSOLE_SOURCE src/TestWin/MatchConsole.cpp src/TestWin/MatchConsole.h)
#add_executable(MatchConsole ${MATCH_CONSOLE_SOURCE})
#target_link_libraries(MatchConsole SiftGPU)
#
#set(SIFTMATCH_CONSOLE_SOURCE src/TestWin/SiftAndMatch.cpp src/TestWin/SiftAndMatch.h)
#add_executable(SiftAndMatch ${SIFTMATCH_CONSOLE_SOURCE})
#target_link_libraries(SiftAndMatch SiftGPU)
