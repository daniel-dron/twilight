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

#include <types.h>

namespace tl {
    u32                  nearest_pow_2( u32 value );
    std::string          read_file_to_string( const char* path );
    std::array<float, 3> hsv_to_rgb( float h, float s, float v );

    constexpr uint32_t comptime_fnv1a_string_32( const char* str ) {
        uint32_t hash = 2166136261u;
        while ( *str ) {
            hash ^= static_cast<uint32_t>( *str );
            hash *= 16777619u;
            str++;
        }
        return hash;
    }

    constexpr glm::vec4 comptime_color( const char* str ) {
        uint32_t hash  = comptime_fnv1a_string_32( str );
        float    hue   = static_cast<float>( hash % 360 );
        auto     color = hsv_to_rgb( hue, 1.0f, 1.0f );
        return glm::vec4{ color[0], color[1], color[2], 1.0f };
    }
} // namespace tl