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