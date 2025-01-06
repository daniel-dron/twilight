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
#include <pch.h>

#include "utils.h"

u32 tl::nearest_pow_2( u32 value ) {
    u32 result = 1;

    // just bruteforce find it
    while ( result * 2 < value ) {
        result *= 2;
    }

    return result;
}

std::string tl::read_file_to_string( const char* path ) {
    std::ifstream file( path, std::ios::ate | std::ios::binary );
    if ( !file.is_open( ) ) {
        return { };
    }

    const size_t file_size = file.tellg( );
    std::string  buffer;
    buffer.resize( file_size );

    file.seekg( 0 );
    file.read( buffer.data( ), file_size );
    file.close( );

    return buffer;
}

std::array<float, 3> tl::hsv_to_rgb( float h, float s, float v ) {
    float c = v * s;
    float x = c * ( 1 - std::abs( std::fmod( h / 60.0f, 2 ) - 1 ) );
    float m = v - c;

    float r, g, b;
    if ( h < 60 ) {
        r = c;
        g = x;
        b = 0;
    }
    else if ( h < 120 ) {
        r = x;
        g = c;
        b = 0;
    }
    else if ( h < 180 ) {
        r = 0;
        g = c;
        b = x;
    }
    else if ( h < 240 ) {
        r = 0;
        g = x;
        b = c;
    }
    else if ( h < 300 ) {
        r = x;
        g = 0;
        b = c;
    }
    else {
        r = c;
        g = 0;
        b = x;
    }

    return { r + m, g + m, b + m };
}