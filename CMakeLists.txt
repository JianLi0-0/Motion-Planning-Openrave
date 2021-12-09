cmake_minimum_required (VERSION 2.6.0)
project(planning_algorithm)
find_package(OpenRAVE REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

include_directories(include)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

add_library(RRTconnect SHARED src/RRTConnect.cpp src/Planner.cpp src/NodeTree.cpp src/KDTree.cpp)
set_target_properties(RRTconnect PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(RRTconnect ${OpenRAVE_LIBRARIES})
install(TARGETS RRTconnect DESTINATION /usr/local/lib/openrave0.53-plugins/)

add_library(BiRRT SHARED src/BiRRT.cpp src/Planner.cpp src/NodeTree.cpp src/KDTree.cpp)
set_target_properties(BiRRT PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(BiRRT ${OpenRAVE_LIBRARIES})
install(TARGETS BiRRT DESTINATION /usr/local/lib/openrave0.53-plugins/)

add_custom_target("uninstall" COMMENT "Uninstall installed files")
add_custom_command(
    TARGET "uninstall"
    POST_BUILD
    COMMENT "Uninstall files with install_manifest.txt"
    COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
            install_manifest.txt to be uninstalled!
)