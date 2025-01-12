cmake_minimum_required(VERSION 3.22)

project(MJPlugin)

include(FetchContent)

FetchContent_Declare(
    mujoco
    GIT_REPOSITORY https://github.com/deepmind/mujoco.git
    GIT_TAG        3.2.5  
)

# Download and make available the MuJoCo dependency
FetchContent_MakeAvailable(mujoco)

add_library(${PROJECT_NAME} STATIC include/mjplugin_template/Plugin.hpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${mujoco_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC 
    mujoco
)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_compile_options(
  ${PROJECT_NAME}
  PRIVATE ${AVX_COMPILE_OPTIONS}
          ${MUJOCO_MACOS_COMPILE_OPTIONS}
          ${EXTRA_COMPILE_OPTIONS}
          ${MUJOCO_CXX_FLAGS}
)

target_link_options(
  ${PROJECT_NAME}
  PRIVATE
  ${MUJOCO_MACOS_LINK_OPTIONS}
  ${EXTRA_LINK_OPTIONS}
)