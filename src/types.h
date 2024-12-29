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

#include <print>

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t  i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

typedef float  f32;
typedef double f64;

constexpr glm::vec3 GLOBAL_UP{ 0.0f, 1.0f, 0.0f };
constexpr glm::vec3 GLOBAL_RIGHT{ -1.0f, 0.0f, 0.0f };
constexpr glm::vec3 GLOBAL_FRONT{ 0.0f, 0.0f, -1.0f };

template<typename T>
T* ptr_to( T&& v ) {
    return &v;
}

template<typename T>
struct Handle {
    u32 handle;
};

inline f64 get_time( ) {
    static auto start_time = std::chrono::high_resolution_clock::now( );
    const auto  now        = std::chrono::high_resolution_clock::now( );
    return std::chrono::duration<double>( now - start_time ).count( );
}

#ifndef NDEBUG
#define DEBUG
#include <vulkan/vk_enum_string_helper.h>
#define VKCALL( x )                                                              \
    do {                                                                         \
        VkResult err = x;                                                        \
        if ( err ) {                                                             \
            std::println( "{} {} Detected Vulkan error: {}", __FILE__, __LINE__, \
                          string_VkResult( err ) );                              \
            abort( );                                                            \
        }                                                                        \
    }                                                                            \
    while ( 0 )
#else
#define VKCALL( x ) x

#endif