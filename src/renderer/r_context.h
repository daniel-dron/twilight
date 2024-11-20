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
#include <vulkan/vulkan_core.h>
#include "r_resources.h"
#include "r_swapchain.h"
#include "types.h"

namespace tl {

    struct FrameData {
        VkCommandPool   pool;
        VkCommandBuffer cmd;
        VkSemaphore     swapchain_semaphore;
        VkSemaphore     render_semaphore;
        VkFence         fence;

        VkQueryPool        query_pool_timestamps = VK_NULL_HANDLE;
        std::array<u64, 2> gpu_timestamps        = { 0 };
    };

    struct Context {
        void initialize( u32 width, u32 height, const std::string& name, struct SDL_Window* window );
        void shutdown( );

        FrameData& get_current_frame( );

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

        // Commands
        VkCommandPool   pool;
        VkCommandBuffer global_cmd;
        VkFence         global_fence;
        Buffer          staging_buffer;

        // Properties
        VkPhysicalDeviceProperties2       properties        = { };
        VkPhysicalDeviceMemoryProperties2 memory_properties = { };

        // Swapchain
        Swapchain swapchain = { };

        u32                                  current_frame = 0;
        static constexpr u32                 frame_overlap = 2;
        std::array<FrameData, frame_overlap> frames;


        inline f64 get_query_time_in_ms( u64 start, u64 end ) const {
            const auto period = properties.properties.limits.timestampPeriod;
            return ( end - start ) * period / 1000000.0f;
        }

    private:
        void _create_device( const std::string& name, struct SDL_Window* window );
        void _create_frames( );
        void _create_global_command( );

        bool m_validation_layers = false;
    };

    extern Context g_ctx;

} // namespace tl