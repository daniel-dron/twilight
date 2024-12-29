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

#include <array>
#include <cassert>
#include <pch.h>
#include <print>
#include <vulkan/vulkan_core.h>
#include "r_resources.h"

#define VOLK_IMPLEMENTATION
#include <Volk/volk.h>
#include <types.h>
#include <utils.h>
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
    swapchain.initialize( width, height, chosen_gpu, device, surface );

    _create_frames( );
    _create_global_command( );

    _create_descriptor_pool( );

    // create global staging buffer (200MB)
    staging_buffer  = create_buffer( 1000 * 1000 * 200, VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, VMA_ALLOCATION_CREATE_MAPPED_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU, false, true );
    readback_buffer = create_buffer( 1000, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, VMA_ALLOCATION_CREATE_MAPPED_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU, false, true );
}

void Context::shutdown( ) {
    swapchain.shutdown( device );

    destroy_buffer( staging_buffer );
    destroy_buffer( readback_buffer );

    // shutdown frames
    for ( auto& frame : frames ) {
        vkDestroyCommandPool( device, frame.pool, nullptr );
        vkDestroySemaphore( device, frame.swapchain_semaphore, nullptr );
        vkDestroySemaphore( device, frame.render_semaphore, nullptr );
        vkDestroyFence( device, frame.fence, nullptr );
        vkDestroyQueryPool( device, frame.query_pool_timestamps, nullptr );
        vkDestroyQueryPool( device, frame.query_pipeline_stats, nullptr );
        destroy_image( frame.color );
        destroy_image( frame.depth );

        for ( u32 i = 0; i < frame.depth_pyramid_levels; i++ ) {
            vkDestroyImageView( device, frame.depth_pyramid_mips[i], nullptr );
        }

        destroy_image( frame.depth_pyramid );
    }


    vkDestroyCommandPool( device, pool, nullptr );
    vkDestroyFence( device, global_fence, nullptr );

    vkDestroyDescriptorPool( device, descriptor_pool, nullptr );

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

void Context::_create_descriptor_pool( ) {
    std::array<VkDescriptorPoolSize, 3> sizes = {
            VkDescriptorPoolSize{ .type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, .descriptorCount = 100 },
            VkDescriptorPoolSize{ .type = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, .descriptorCount = 100 },
            VkDescriptorPoolSize{ .type = VK_DESCRIPTOR_TYPE_SAMPLER, .descriptorCount = 10 } };
    descriptor_pool = create_descriptor_pool( sizes.data( ), sizes.size( ), 100 );
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
    VkPhysicalDeviceVulkan13Features features_13{
            .sType            = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES,
            .synchronization2 = true,
            .dynamicRendering = true,
            .maintenance4     = true,
    };
    VkPhysicalDeviceVulkan12Features features_12{
            .sType                   = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES,
            .storageBuffer8BitAccess = true,
            .shaderInt8              = true,
            .runtimeDescriptorArray  = true,
            .samplerFilterMinmax     = true,
            .hostQueryReset          = true,
            .bufferDeviceAddress     = true,
    };
    VkPhysicalDeviceVulkan11Features features_11{
            .sType                         = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES,
            .variablePointersStorageBuffer = VK_TRUE,
            .variablePointers              = VK_TRUE,
            .shaderDrawParameters          = VK_TRUE,
    };
    VkPhysicalDeviceFeatures features{
            .multiDrawIndirect       = VK_TRUE,
            .pipelineStatisticsQuery = VK_TRUE,
            .samplerAnisotropy       = VK_TRUE,
            .shaderInt64             = VK_TRUE,
            .fillModeNonSolid        = VK_TRUE,
    };


    PhysicalDeviceSelector selector{ instance };

    auto physical_device = selector.set_minimum_version( 1, 3 )
                                   .set_required_features_13( features_13 )
                                   .set_required_features_12( features_12 )
                                   .set_required_features_11( features_11 )
                                   .set_required_features( features )
                                   .add_required_extension( VK_EXT_MESH_SHADER_EXTENSION_NAME )
                                   .add_required_extension( VK_EXT_SAMPLER_FILTER_MINMAX_EXTENSION_NAME )
                                   .add_required_extension( VK_NV_COMPUTE_SHADER_DERIVATIVES_EXTENSION_NAME )
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
            .sType             = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MESH_SHADER_FEATURES_EXT,
            .taskShader        = VK_TRUE,
            .meshShader        = VK_TRUE,
            .meshShaderQueries = VK_TRUE,
    };

    VkPhysicalDeviceComputeShaderDerivativesFeaturesNV compute_derivatives_features{
            .sType                        = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_COMPUTE_SHADER_DERIVATIVES_FEATURES_NV,
            .computeDerivativeGroupQuads  = VK_TRUE,
            .computeDerivativeGroupLinear = VK_TRUE };

    DeviceBuilder device_builder{ physical_device.value( ) };
    device_builder.add_pNext( &mesh_features );
    device_builder.add_pNext( &compute_derivatives_features );
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

        // Create timer pools
        VkQueryPoolCreateInfo query_pool_info = {
                .sType      = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO,
                .pNext      = nullptr,
                .queryType  = VK_QUERY_TYPE_TIMESTAMP,
                .queryCount = u32( frame.gpu_timestamps.size( ) ) };
        VKCALL( vkCreateQueryPool( device, &query_pool_info, nullptr, &frame.query_pool_timestamps ) );
        vkResetQueryPool( device, frame.query_pool_timestamps, 0, frame.gpu_timestamps.size( ) );
    }

    _create_images( );
}


void Context::_create_images( ) {
    for ( auto& frame : frames ) {
        frame.color      = create_image( swapchain.width, swapchain.height, VK_FORMAT_R32G32B32A32_SFLOAT, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, 1 );
        frame.color.view = create_view( frame.color );

        frame.depth      = create_image( swapchain.width, swapchain.height, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, 1 );
        frame.depth.view = create_view( frame.depth );

        frame.depth_pyramid_size   = nearest_pow_2( swapchain.width );
        frame.depth_pyramid_levels = get_mip_count( frame.depth_pyramid_size, frame.depth_pyramid_size );
        frame.depth_pyramid        = create_image( frame.depth_pyramid_size, frame.depth_pyramid_size, VK_FORMAT_R32_SFLOAT, VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, frame.depth_pyramid_levels );
        frame.depth_pyramid.view   = create_view( frame.depth_pyramid );
        for ( u32 i = 0; i < frame.depth_pyramid_levels; i++ ) {
            frame.depth_pyramid_mips[i] = create_view( frame.depth_pyramid, i, 1 );
            assert( frame.depth_pyramid_mips[i] != VK_NULL_HANDLE );
        }
    }
}

FrameData& Context::get_current_frame( ) {
    return frames.at( current_frame % frame_overlap );
}

u32 Context::get_current_frame_index( ) {
    return current_frame % frame_overlap;
}

void Context::resize( u32 width, u32 height, VkDevice device, VkSurfaceKHR surface ) {
    for ( auto& frame : frames ) {
        destroy_image( frame.depth );
        destroy_image( frame.color );
        destroy_image( frame.depth_pyramid );
    }

    _create_images( );

    swapchain.resize( width, height, chosen_gpu, device, surface );
}