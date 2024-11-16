# Exclude warnings from the current directory
set_directory_properties(
    PROPERTIES
    COMPILE_FLAGS "-w"
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(SDL EXCLUDE_FROM_ALL)

find_package(Vulkan REQUIRED)

# VMA is a header only lib
add_library(vma INTERFACE)
target_include_directories(vma INTERFACE vma)

# VkBootstrap
add_library(vkbootstrap STATIC)
target_sources(vkbootstrap PRIVATE 
  vkbootstrap/VkBootstrap.h
  vkbootstrap/VkBootstrap.cpp
)
target_include_directories(vkbootstrap PUBLIC vkbootstrap ${Vulkan_INCLUDE_DIRS})
target_link_directories(vkbootstrap PUBLIC Vulkan::Vulkan ${CMAKE_DL_LIBS})
set_property(TARGET vkbootstrap PROPERTY CXX_STANDARD 20)

# STB IMAGE
add_library(stb_image INTERFACE)
target_include_directories(stb_image INTERFACE stb_image)

# Assimp
add_subdirectory(assimp EXCLUDE_FROM_ALL)

# FMT
add_subdirectory(fmt EXCLUDE_FROM_ALL)

# Meshoptimizer
# add_subdirectory(meshoptimizer)

# GLM
add_library(glm INTERFACE)
target_include_directories(glm INTERFACE glm)

# ImGui
add_library(imgui STATIC)
target_sources(imgui PRIVATE 
  imgui/imgui.h
  imgui/imgui.cpp

  imgui/imgui_demo.cpp
  imgui/imgui_draw.cpp
  imgui/imgui_widgets.cpp
  imgui/imgui_tables.cpp
  imgui/imgui_impl_vulkan.cpp
  imgui/imgui_impl_sdl2.cpp
)
target_include_directories(imgui PUBLIC imgui)
target_link_libraries(imgui PUBLIC Vulkan::Vulkan SDL2::SDL2)

# Volk
add_library(volk STATIC)
target_sources(volk PRIVATE volk/volk.h volk/volk.c)
target_include_directories(volk PUBLIC volk ${Vulkan_INCLUDE_DIRS})

# Tracy Profiler
add_library(tracy STATIC)
target_sources(tracy PRIVATE 
    tracy/TracyClient.cpp
)
target_compile_definitions(tracy PUBLIC TRACY_ENABLE)
target_include_directories(tracy PUBLIC tracy)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
# set_target_properties(sdl_headers_copy tracy meshoptimizer assimp glm vma fmt imgui SDL2 vkbootstrap volk PROPERTIES FOLDER Dependencies)
set_target_properties(sdl_headers_copy tracy assimp glm vma fmt imgui SDL2 vkbootstrap volk PROPERTIES FOLDER Dependencies)
