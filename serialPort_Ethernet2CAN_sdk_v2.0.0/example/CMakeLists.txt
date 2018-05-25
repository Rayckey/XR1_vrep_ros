cmake_minimum_required(VERSION 3.0)
project(example)

SET(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../sdk/include)

SET(EXAMPLE_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/src/lookupActuators.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/monitorActuator.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/operateActuator.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/tuneActuator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/homingActuator.cpp
)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(WIN32)
	SET(LIB_DIR_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/../sdk/lib/windows_x64/release)
	SET(LIB_DIR_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/../sdk/lib/windows_x64/debug)
	link_directories(${LIB_DIR_RELEASE})
	link_directories(${LIB_DIR_DEBUG})
elseif(UNIX)
	SET(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../sdk/lib/linux_x86_64)
	link_directories(${LIB_DIR})
endif()

include_directories(${INCLUDE_DIR})
include_directories(${INCLUDE_DIR}/QtCore)
include_directories(${INCLUDE_DIR}/QtNetwork)

add_compile_options(-std=c++11)
add_compile_options(-fPIC)
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-pthread)


foreach(EXAMPLE ${EXAMPLE_SOURCES})
    get_filename_component(EX_NAME ${EXAMPLE} NAME_WE)
    add_executable(${EX_NAME} ${EXAMPLE})
    if(WIN32)
        target_link_libraries(${EX_NAME} debug actuatorControllerd optimized actuatorController)
    elseif(UNIX)
        target_link_libraries(${EX_NAME} actuatorController pthread)
    endif()
endforeach(EXAMPLE ${EXAMPLE_SOURCES})

#if(WIN32)
#	target_link_libraries(MyTest debug actuatorControllerd optimized actuatorController)
#elseif(UNIX)
#	target_link_libraries(MyTest actuatorController pthread)
#endif()
