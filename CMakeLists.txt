cmake_minimum_required(VERSION 3.8)
project(usad_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenGL_GL_PREFERENCE GLVND)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ira_interfaces REQUIRED)  
find_package(SDL2 REQUIRED)

include_directories(${SDL2_INCLUDE_DIRS} 
                    ${CMAKE_CURRENT_SOURCE_DIR}/imgui-1.90
                    ${CMAKE_CURRENT_SOURCE_DIR}/imgui-knobs-main)

add_executable(usad_gui src/usad_gui.cpp 
                        imgui-1.90/imgui_draw.cpp
                        imgui-1.90/imgui_tables.cpp
                        imgui-1.90/imgui_widgets.cpp
                        imgui-1.90/imgui.cpp
                        imgui-1.90/backends/imgui_impl_sdl2.cpp
                        imgui-1.90/backends/imgui_impl_sdlrenderer2.cpp 
                        imgui-knobs-main/imgui-knobs.cpp)
ament_target_dependencies(usad_gui 
                          rclcpp
                          std_msgs 
                          tf2 
                          tf2_ros 
                          geometry_msgs 
                          nav_msgs
                          ira_interfaces)
target_include_directories(usad_gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  target_link_libraries(usad_gui ${SDL2_LIBRARIES} ${OPENGL_LIBRARIES})
target_compile_features(usad_gui PUBLIC c_std_99 cxx_std_17)

install(TARGETS usad_gui DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
