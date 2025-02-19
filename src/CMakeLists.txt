# /*****************************************************************************************
# ******************************************************************************************
# **                                                                                      **
# **                                                                                      **
# **     ███      ▄█     █▄   ▄█   ▄█        ▄█     ▄██████▄     ▄█    █▄        ███      **
# ** ▀█████████▄ ███     ███ ███  ███       ███    ███    ███   ███    ███   ▀█████████▄  **
# **    ▀███▀▀██ ███     ███ ███▌ ███       ███▌   ███    █▀    ███    ███      ▀███▀▀██  **
# **     ███   ▀ ███     ███ ███▌ ███       ███▌  ▄███         ▄███▄▄▄▄███▄▄     ███   ▀  **
# **     ███     ███     ███ ███▌ ███       ███▌ ▀▀███ ████▄  ▀▀███▀▀▀▀███▀      ███      **
# **     ███     ███     ███ ███  ███       ███    ███    ███   ███    ███       ███      **
# **     ███     ███ ▄█▄ ███ ███  ███▌    ▄ ███    ███    ███   ███    ███       ███      **
# **    ▄████▀    ▀███▀███▀  █▀   █████▄▄██ █▀     ████████▀    ███    █▀       ▄████▀    **
# **                              ▀                                                       **
# **                                                                                      **
# **                      Copyright (c) 2024-present Daniel Dron                          **
# **                                                                                      **
# **                  This software is released under the MIT License.                    **
# **                       https://opensource.org/licenses/MIT                            **
# **                                                                                      **
# ******************************************************************************************
# *****************************************************************************************/

cmake_minimum_required(VERSION 3.8.0 FATAL_ERROR)

project(TwilightEngine VERSION 0.1.0 LANGUAGES CXX C)

file(GLOB_RECURSE ALL_SRCS *.cpp *.hpp *.h)

add_executable(TwilightEngine ${ALL_SRCS})

# Enforce latest C++23 features (We will not be using most of them, but there are a few time savers)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
target_compile_features(TwilightEngine PRIVATE cxx_std_23)
set_target_properties(TwilightEngine PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:TwilightEngine>"
)
target_compile_definitions(TwilightEngine PUBLIC GLM_FORCE_DEPTH_ZERO_TO_ONE)
target_compile_definitions(TwilightEngine PUBLIC GLM_FORCE_RIGHT_HANDED)

# Set some preprocessor flags relative to what Visual Studio run configuration is being compiled/used
add_compile_definitions(
        $<$<CONFIG:Debug,RelWithDebInfo>:ENABLE_DEBUG_FEATURES>
)

# Set twilight engine folder structure for Visual Studio Solutions
foreach (SRC_FILE ${ALL_SRCS})
    file(RELATIVE_PATH SRC_REL_PATH ${PROJECT_SOURCE_DIR} ${SRC_FILE})
    get_filename_component(SRC_REL_DIR ${SRC_REL_PATH} DIRECTORY)

    if (SRC_REL_DIR)
        string(REPLACE "/" "\\" VS_FOLDER "src\\${SRC_REL_DIR}")
        source_group("${VS_FOLDER}" FILES ${SRC_FILE})
    else ()
        source_group("src" FILES ${SRC_FILE})
    endif ()
endforeach ()
target_include_directories(TwilightEngine PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Pre-Compiled Headers for faster compilation
target_precompile_headers(TwilightEngine PRIVATE pch.h)

target_link_libraries(TwilightEngine PUBLIC tracy meshoptimizer assimp glm vma fmt::fmt vkbootstrap SDL2::SDL2 stb_image imgui nlohmann_json::nlohmann_json)
# target_link_libraries(TwilightEngine PUBLIC tracy assimp glm vma fmt::fmt vkbootstrap SDL2::SDL2 stb_image imgui)

# SHADERS
set(SHADER_SOURCE_DIR "${PROJECT_SOURCE_DIR}/../shaders/")
set(SHADER_BINARY_DIR "${PROJECT_SOURCE_DIR}/../bin/shaders/")
file(GLOB_RECURSE GLSL_SHADERS ${SHADER_SOURCE_DIR}/*.vert.slang ${SHADER_SOURCE_DIR}/*.frag.slang ${SHADER_SOURCE_DIR}/*.mesh.slang ${SHADER_SOURCE_DIR}/*.task.slang ${SHADER_SOURCE_DIR}/*.comp.slang)
file(GLOB_RECURSE ALL_GLSL_SHADER_FILES ${SHADER_SOURCE_DIR}/*.slang)

message(STATUS ${ALL_GLSL_SHADER_FILES})

set(SLANG_COMPILER "C:/Tools/slangc/bin/slangc") # Hardcoded shameful for now....
message(${SLANG_COMPILER})

foreach (GLSL ${GLSL_SHADERS})
    message(STATUS "BUILDING SHADER")
    get_filename_component(FILE_NAME ${GLSL} NAME)
    set(SPIRV_COMBINED "${SHADER_BINARY_DIR}/${FILE_NAME}.spv")
    set(SPRIV_COMBINED_JSON "${SHADER_BINARY_DIR}/${FILE_NAME}.spv.json")

    # Determine shader stage from extension
    if(${GLSL} MATCHES "\.task\.slang$")
        set(STAGE_FLAG amplification)
    elseif(${GLSL} MATCHES "\.mesh\.slang$")
        set(STAGE_FLAG mesh)
    elseif(${GLSL} MATCHES "\.vert\.slang$")
        set(STAGE_FLAG vertex)
    elseif(${GLSL} MATCHES "\.frag\.slang$")
        set(STAGE_FLAG fragment)
    elseif(${GLSL} MATCHES "\.comp\.slang$")
        set(STAGE_FLAG compute)
    else()
        set(STAGE_FLAG)
    endif()

    # Construct the full command as a string for echo
    set(COMMAND_STRING "${SLANG_COMPILER} ${GLSL} -stage ${STAGE_FLAG} -g -force-glsl-scalar-layout -target spirv -reflection-json ${SPRIV_COMBINED_JSON} -o ${SPIRV_COMBINED}")
    
    message(STATUS ${GLSL})
    add_custom_command(
            OUTPUT ${SPIRV_COMBINED}
            COMMAND ${CMAKE_COMMAND} -E echo "Running: ${COMMAND_STRING}"
            COMMAND ${SLANG_COMPILER} ${GLSL} -stage ${STAGE_FLAG} -g -force-glsl-scalar-layout -target spirv -reflection-json ${SPRIV_COMBINED_JSON} -o ${SPIRV_COMBINED}

            DEPENDS ${GLSL}
            BYPRODUCTS ${SPIRV_VERTEX} ${SPIRV_PIXEL}
    )
    list(APPEND SPIRV_BINARY_FILES ${SPIRV_COMBINED})
endforeach (GLSL)

#
# Black magic to create a visual studio project inside the solution and for intellisense to work properly
# with proper filter paths to replica actual directories.
#
add_library(Shaders STATIC ${ALL_GLSL_SHADER_FILES})
set_target_properties(Shaders PROPERTIES LINKER_LANGUAGE CXX)
set_source_files_properties(
        ${ALL_GLSL_SHADER_FILES}
        PROPERTIES
        VS_TOOL_OVERRIDE "None"  # Prevents VS from trying to compile them
)

add_custom_target(
        ShadersCompile
        DEPENDS ${SPIRV_BINARY_FILES}
)
add_dependencies(Shaders ShadersCompile)
add_dependencies(TwilightEngine ShadersCompile)

foreach (SHADER_FILE ${ALL_GLSL_SHADER_FILES})
    file(RELATIVE_PATH SHADER_REL_PATH ${SHADER_SOURCE_DIR} ${SHADER_FILE})
    get_filename_component(SHADER_REL_DIR ${SHADER_REL_PATH} DIRECTORY)

    if (SHADER_REL_DIR)
        string(REPLACE "/" "\\" VS_FOLDER "Shaders\\${SHADER_REL_DIR}")
        source_group("${VS_FOLDER}" FILES ${SHADER_FILE})
    else ()
        source_group("Shaders" FILES ${SHADER_FILE})
    endif ()
endforeach ()

set_target_properties(ShadersCompile zlibstatic PROPERTIES FOLDER Dependencies)