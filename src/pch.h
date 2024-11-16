/******************************************************************************
******************************************************************************
**                                                                           **
**                             Twilight Engine                               **
**                                                                           **
**                  Copyright (c) 2024-present Daniel Dron                   **
**                                                                           **
**            This software is released under the MIT License.               **
**                 https://opensource.org/licenses/MIT                       **
**                                                                           **
******************************************************************************
******************************************************************************/

#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <expected>
#include <format>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <random>
#include <span>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// Third Party headers

// fmt
#include <fmt/core.h>

// Assimp
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/texture.h>

// STB
#include <stb_image.h>

// Volk
#include <Volk/volk.h>

// Vulkan
#include <VkBootstrap.h>
#include <vk_mem_alloc.h>

// SDL
#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_vulkan.h>
#include <SDL_events.h>
#include <SDL_stdinc.h>
#include <SDL_video.h>

// Imgui
#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_vulkan.h>
#include <imgui_internal.h>

// GLM
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/integer.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/packing.hpp>

// Tracy
#include <tracy/Tracy.hpp>