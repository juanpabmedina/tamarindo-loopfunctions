# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
link_directories(
	${CMAKE_BINARY_DIR}/src
    )

add_library(communication_homing_loopfunc SHARED CommHomLoopFunc.h CommHomLoopFunc.cpp)
target_link_libraries(communication_homing_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(communication_tsk_loopfunc SHARED CommTskLoopFunc.h CommTskLoopFunc.cpp)
target_link_libraries(communication_tsk_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(communication_aggregation_loopfunc SHARED CommAggLoopFunc.h CommAggLoopFunc.cpp)
target_link_libraries(communication_aggregation_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)