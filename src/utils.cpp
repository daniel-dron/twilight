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