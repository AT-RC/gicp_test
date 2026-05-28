cmake_minimum_required(VERSION 3.10)
project(small_gicp_relocalization)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

# 1. 创建共享库 (包含节点逻辑)
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/small_gicp_relocalization.cpp
)
target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)

# 2. 创建自定义可执行文件 (包含我们的多线程 main)
add_executable(${PROJECT_NAME}_node src/small_gicp_relocalization_main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# 3. 注册组件 (仅作为插件，不让它生成重复的 node)
# 我们手动调用底层宏，绕过 Humble 强制要求 EXECUTABLE 的限制
rclcpp_components_register_nodes(${PROJECT_NAME} "small_gicp_relocalization::SmallGicpRelocalizationNode")

#############
## Install ##
#############

# 安装可执行文件到 lib/small_gicp_relocalization/
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
