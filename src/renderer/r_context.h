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
#include "r_swapchain.h"
#include "types.h"

namespace tl {

    struct Context {
        void initialize( u32 width, u32 height, const std::string& name, struct SDL_Window* window );
        void shutdown( );

        // Vulkan handles
        VkInstance               instance        = VK_NULL_HANDLE;
        VkSurfaceKHR             surface         = VK_NULL_HANDLE;
        VkPhysicalDevice         chosen_gpu      = VK_NULL_HANDLE;
        VkDevice                 device          = VK_NULL_HANDLE;
        VkDebugUtilsMessengerEXT debug_messenger = VK_NULL_HANDLE;

        // Allocator
        VmaAllocator allocator = { };

        // Queues
        VkQueue graphics_queue        = VK_NULL_HANDLE;
        u32     graphics_queue_family = 0;
        VkQueue compute_queue         = VK_NULL_HANDLE;
        u32     compute_queue_family  = 0;

        // Properties
        VkPhysicalDeviceProperties2       properties        = { };
        VkPhysicalDeviceMemoryProperties2 memory_properties = { };

        // Swapchain
        Swapchain swapchain = { };

    private:
        void _create_device( const std::string& name, struct SDL_Window* window );

        bool m_validation_layers = false;
    };

} // namespace tl