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

#include <cassert>
#include <pch.h>
#include <print>
#include <vulkan/vulkan_core.h>
#include "r_resources.h"

#define VOLK_IMPLEMENTATION
#include <Volk/volk.h>
#include <types.h>
#include "SDL_vulkan.h"
#include "VkBootstrap.h"
#include "r_context.h"


#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>

using namespace tl;
using namespace vkb;

namespace tl {
    Context g_ctx = { };
}

void Context::initialize( u32 width, u32 height, const std::string& name, struct SDL_Window* window ) {

// Enable validation in debug mode
#ifdef DEBUG
    m_validation_layers = true;
    std::println( "Twilight running in debug mode. Enabling vulkan validation layers." );
#endif

    _create_device( name, window );
    _create_frames( );
    _create_global_command( );

    // Swapchain
    swapchain.initialize( width, height, chosen_gpu, device, surface );
}

void Context::shutdown( ) {
    swapchain.shutdown( device );

    // shutdown frames
    for ( auto& frame : frames ) {
        vkDestroyCommandPool( device, frame.pool, nullptr );
        vkDestroySemaphore( device, frame.swapchain_semaphore, nullptr );
        vkDestroySemaphore( device, frame.render_semaphore, nullptr );
        vkDestroyFence( device, frame.fence, nullptr );
    }

    vkDestroyCommandPool( device, pool, nullptr );
    vkDestroyFence( device, global_fence, nullptr );

    vmaDestroyAllocator( allocator );

    if ( debug_messenger ) {
        vkDestroyDebugUtilsMessengerEXT( instance, debug_messenger, nullptr );
    }
    vkDestroySurfaceKHR( instance, surface, nullptr );
    vkDestroyDevice( device, nullptr );
    vkDestroyInstance( instance, nullptr );
}

void Context::_create_global_command( ) {
    const VkCommandPoolCreateInfo info{
            .sType            = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
            .pNext            = nullptr,
            .flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT,
            .queueFamilyIndex = graphics_queue_family };
    VKCALL( vkCreateCommandPool( device, &info, nullptr, &pool ) );

    const VkCommandBufferAllocateInfo cmd_info = {
            .sType              = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
            .pNext              = nullptr,
            .commandPool        = pool,
            .level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
            .commandBufferCount = 1,
    };
    VKCALL( vkAllocateCommandBuffers( device, &cmd_info, &global_cmd ) );

    global_fence = create_fence( VK_FENCE_CREATE_SIGNALED_BIT );
}

void Context::_create_device( const std::string& name, struct SDL_Window* window ) {
    volkInitialize( );

    InstanceBuilder builder;

    // Instance
    auto prototype = builder.set_app_name( name.c_str( ) )
                             .request_validation_layers( m_validation_layers )
#ifdef DEBUG
                             .use_default_debug_messenger( )
#endif
                             .require_api_version( 1, 3, 0 )
                             .build( );
    assert( prototype.has_value( ) );
    auto& instance = prototype.value( );
    this->instance = instance;
    assert( instance != VK_NULL_HANDLE );

    volkLoadInstance( this->instance );

    this->debug_messenger = instance.debug_messenger;

    // Surface
    SDL_Vulkan_CreateSurface( window, instance, &this->surface );
    assert( this->surface != VK_NULL_HANDLE );

    // Physical device
    VkPhysicalDeviceVulkan13Features features_13{ .sType            = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES,
                                                  .synchronization2 = true,
                                                  .dynamicRendering = true,
                                                  .maintenance4     = true };
    VkPhysicalDeviceVulkan12Features features_12{
            .sType                   = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES,
            .storageBuffer8BitAccess = true,
            .bufferDeviceAddress     = true };
    VkPhysicalDeviceVulkan11Features features_11{ .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES };
    VkPhysicalDeviceFeatures         features{ };


    PhysicalDeviceSelector selector{ instance };

    auto physical_device = selector.set_minimum_version( 1, 3 )
                                   .set_required_features_13( features_13 )
                                   .set_required_features_12( features_12 )
                                   .set_required_features_11( features_11 )
                                   .set_required_features( features )
                                   .add_required_extension( VK_EXT_MESH_SHADER_EXTENSION_NAME )
                                   .set_surface( this->surface )
                                   .select( );
    assert( physical_device.has_value( ) );
    this->chosen_gpu = physical_device.value( );

    // Get Properties
    VkPhysicalDeviceProperties2 properties{ .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
    vkGetPhysicalDeviceProperties2( this->chosen_gpu, &properties );
    this->properties = properties;

    VkPhysicalDeviceMemoryProperties2 memory_properties{ .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MEMORY_PROPERTIES_2 };
    vkGetPhysicalDeviceMemoryProperties2( this->chosen_gpu, &memory_properties );
    this->memory_properties = memory_properties;

    // Device
    VkPhysicalDeviceMeshShaderFeaturesEXT mesh_features{
            .sType      = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MESH_SHADER_FEATURES_EXT,
            .meshShader = VK_TRUE };

    DeviceBuilder device_builder{ physical_device.value( ) };
    device_builder.add_pNext( &mesh_features );
    auto device = device_builder.build( );
    assert( device.has_value( ) );
    this->device = device->device;
    volkLoadDevice( this->device );

    // Queues
    graphics_queue        = device->get_queue( QueueType::graphics ).value( );
    graphics_queue_family = device->get_queue_index( QueueType::graphics ).value( );
    assert( graphics_queue != VK_NULL_HANDLE );

    compute_queue        = device->get_queue( QueueType::compute ).value( );
    compute_queue_family = device->get_queue_index( QueueType::compute ).value( );
    assert( compute_queue != VK_NULL_HANDLE );

    // VMA allocator
    VmaVulkanFunctions vma_vulkan_func{
            .vkGetInstanceProcAddr = vkGetInstanceProcAddr,
            .vkGetDeviceProcAddr   = vkGetDeviceProcAddr };

    const VmaAllocatorCreateInfo info = {
            .flags            = VMA_ALLOCATOR_CREATE_BUFFER_DEVICE_ADDRESS_BIT,
            .physicalDevice   = this->chosen_gpu,
            .device           = this->device,
            .pVulkanFunctions = &vma_vulkan_func,
            .instance         = instance,
    };

    auto res = vmaCreateAllocator( &info, &this->allocator );
    assert( res == VK_SUCCESS );
}

void Context::_create_frames( ) {
    for ( auto& frame : frames ) {
        const VkCommandPoolCreateInfo info{
                .sType            = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
                .pNext            = nullptr,
                .flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT,
                .queueFamilyIndex = graphics_queue_family };
        VKCALL( vkCreateCommandPool( device, &info, nullptr, &frame.pool ) );

        const VkCommandBufferAllocateInfo cmd_info = {
                .sType              = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
                .pNext              = nullptr,
                .commandPool        = frame.pool,
                .level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
                .commandBufferCount = 1,
        };
        VKCALL( vkAllocateCommandBuffers( device, &cmd_info, &frame.cmd ) );

        frame.fence               = create_fence( VK_FENCE_CREATE_SIGNALED_BIT );
        frame.swapchain_semaphore = create_semaphore( );
        frame.render_semaphore    = create_semaphore( );
    }
}

FrameData& Context::get_current_frame( ) {
    return frames.at( current_frame % frame_overlap );
}