cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

# carica il package catkin bloccando il processo se non sono presenti i components dichiarati
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

# specifica le cartelle contenenti gli headers
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/beginner_tutorials.cpp
# )

# dichiarazione eseguibile
add_executable(talker_node src/talker.cpp)
add_executable(listener_node src/listener.cpp)

# specifica dipendenze eseguibile
add_dependencies(talker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(listener_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# specifica librerie da linkare  
target_link_libraries(talker_node ${catkin_LIBRARIES})
target_link_libraries(listener_node ${catkin_LIBRARIES})
