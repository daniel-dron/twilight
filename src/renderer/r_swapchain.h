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

#include <Volk/volk.h>
#include "types.h"

namespace tl {

    struct Swapchain {
        VkSwapchainKHR       swapchain = VK_NULL_HANDLE;
        std::vector<VkImage> images;
        u32                  width, height;
        u32                  image_count;

        void initialize( u32 width, u32 height, VkPhysicalDevice physical_device, VkDevice device, VkSurfaceKHR surface );
        void shutdown( VkDevice device );

        void resize( u32 width, u32 height, VkPhysicalDevice physical_device, VkDevice device, VkSurfaceKHR surface );

    private:
        void create( u32 width, u32 height, VkPhysicalDevice physical_device, VkDevice device, VkSurfaceKHR surface, VkSwapchainKHR old_swapchain);
    };

} // namespace tl