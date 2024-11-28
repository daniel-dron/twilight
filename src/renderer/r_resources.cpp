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
#include <vulkan/vulkan_core.h>

#include "r_context.h"
#include "r_resources.h"
#include "renderer/r_context.h"
#include "types.h"

using namespace tl;

VkFence tl::create_fence( const VkFenceCreateFlags flags ) {
    VkFence fence;

    const VkFenceCreateInfo info = {
            .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
            .pNext = nullptr,
            .flags = flags,
    };

    VKCALL( vkCreateFence( g_ctx.device, &info, nullptr, &fence ) );
    return fence;
}

VkSemaphore tl::create_semaphore( const VkSemaphoreCreateFlags flags ) {
    VkSemaphore semaphore;

    const VkSemaphoreCreateInfo info{
            .sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO,
            .pNext = nullptr,
            .flags = flags };

    VKCALL( vkCreateSemaphore( g_ctx.device, &info, nullptr, &semaphore ) );
    return semaphore;
}

void tl::begin_command( VkCommandBuffer cmd, const VkCommandBufferUsageFlags flags ) {
    const VkCommandBufferBeginInfo info = {
            .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
            .pNext = nullptr,
            .flags = flags };

    VKCALL( vkBeginCommandBuffer( cmd, &info ) );
}

void tl::submit_graphics_command( VkCommandBuffer cmd, const VkPipelineStageFlags2 wait_stage, const VkPipelineStageFlags2 signal_stage, const VkSemaphore wait_semaphore, const VkSemaphore render_semaphore, const VkFence fence ) {
    const VkCommandBufferSubmitInfo buffer_info{
            .sType         = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO,
            .pNext         = nullptr,
            .commandBuffer = cmd,
            .deviceMask    = 0,
    };

    const VkSemaphoreSubmitInfo wait_info{
            .sType       = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
            .pNext       = nullptr,
            .semaphore   = wait_semaphore,
            .value       = 1,
            .stageMask   = wait_stage,
            .deviceIndex = 0 };

    const VkSemaphoreSubmitInfo signal_info{
            .sType       = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
            .pNext       = nullptr,
            .semaphore   = render_semaphore,
            .value       = 1,
            .stageMask   = signal_stage,
            .deviceIndex = 0 };

    const VkSubmitInfo2 info{
            .sType                    = VK_STRUCTURE_TYPE_SUBMIT_INFO_2,
            .pNext                    = nullptr,
            .flags                    = 0,
            .waitSemaphoreInfoCount   = 1u,
            .pWaitSemaphoreInfos      = &wait_info,
            .commandBufferInfoCount   = 1,
            .pCommandBufferInfos      = &buffer_info,
            .signalSemaphoreInfoCount = 1u,
            .pSignalSemaphoreInfos    = &signal_info };

    VKCALL( vkQueueSubmit2( g_ctx.graphics_queue, 1, &info, fence ) );
}

void tl::submit_command( VkCommandBuffer cmd, VkQueue queue, const VkFence fence ) {
    const VkCommandBufferSubmitInfo buffer_info{
            .sType         = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO,
            .pNext         = nullptr,
            .commandBuffer = cmd,
            .deviceMask    = 0,
    };

    const VkSubmitInfo2 info{
            .sType                    = VK_STRUCTURE_TYPE_SUBMIT_INFO_2,
            .pNext                    = nullptr,
            .flags                    = 0,
            .waitSemaphoreInfoCount   = 0u,
            .pWaitSemaphoreInfos      = nullptr,
            .commandBufferInfoCount   = 1,
            .pCommandBufferInfos      = &buffer_info,
            .signalSemaphoreInfoCount = 0,
            .pSignalSemaphoreInfos    = nullptr };

    VKCALL( vkQueueSubmit2( queue, 1, &info, fence ) );
}

VkRenderingAttachmentInfo tl::attachment( const VkImageView view, const VkClearValue* clear, const VkImageLayout layout ) {
    const VkRenderingAttachmentInfo color_attachment{
            .sType       = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
            .pNext       = nullptr,
            .imageView   = view,
            .imageLayout = layout,
            .loadOp      = clear ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD,
            .storeOp     = VK_ATTACHMENT_STORE_OP_STORE,
            .clearValue  = clear ? *clear : VkClearValue{ },
    };

    return color_attachment;
}

void tl::image_barrier( VkCommandBuffer cmd, VkImage image, VkPipelineStageFlags2 src_stage, VkAccessFlags2 src_access, VkImageLayout old_layout, VkPipelineStageFlags2 dst_stage, VkAccessFlags2 dst_access, VkImageLayout new_layout, VkImageAspectFlags aspect_mask, u32 base_mip, u32 level_count ) {
    VkImageMemoryBarrier2 image_barrier{
            .sType         = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2,
            .pNext         = nullptr,
            .srcStageMask  = src_stage,
            .srcAccessMask = src_access,
            .dstStageMask  = dst_stage,
            .dstAccessMask = dst_access,

            .oldLayout = old_layout,
            .newLayout = new_layout,

            .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,

            .image = image,

            .subresourceRange = {
                    .aspectMask     = aspect_mask,
                    .baseMipLevel   = 0,
                    .levelCount     = VK_REMAINING_MIP_LEVELS,
                    .baseArrayLayer = 0,
                    .layerCount     = VK_REMAINING_ARRAY_LAYERS,
            },
    };

    const VkDependencyInfo dependency_info{
            .sType                   = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
            .pNext                   = nullptr,
            .imageMemoryBarrierCount = 1,
            .pImageMemoryBarriers    = &image_barrier,
    };

    vkCmdPipelineBarrier2( cmd, &dependency_info );
}

void tl::buffer_barrier( VkCommandBuffer cmd, VkBuffer buffer, u64 size, VkPipelineStageFlags2 src_stage, VkAccessFlags2 src_access, VkPipelineStageFlags2 dst_stage, VkAccessFlags2 dst_access ) {
    VkBufferMemoryBarrier2 buffer_barrier{
            .sType         = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2,
            .pNext         = nullptr,
            .srcStageMask  = src_stage,
            .srcAccessMask = src_access,
            .dstStageMask  = dst_stage,
            .dstAccessMask = dst_access,
            .buffer        = buffer,
            .size          = size,
    };

    const VkDependencyInfo dependency_info{
            .sType                    = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
            .pNext                    = nullptr,
            .bufferMemoryBarrierCount = 1,
            .pBufferMemoryBarriers    = &buffer_barrier,
    };
    vkCmdPipelineBarrier2( cmd, &dependency_info );
}

VkDescriptorPool tl::create_descriptor_pool( const VkDescriptorPoolSize* sizes, u32 pool_size, u32 max_sets ) {
    VkDescriptorPool pool{ };

    const VkDescriptorPoolCreateInfo info{
            .sType         = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,
            .pNext         = nullptr,
            .flags         = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT,
            .maxSets       = max_sets,
            .poolSizeCount = pool_size,
            .pPoolSizes    = sizes };
    VKCALL( vkCreateDescriptorPool( g_ctx.device, &info, nullptr, &pool ) );

    return pool;
}

VkDescriptorSetLayout tl::create_descriptor_layout( const VkDescriptorSetLayoutBinding* bindings, uint32_t binding_count ) {
    VkDescriptorSetLayout layout{ };

    const VkDescriptorSetLayoutCreateInfo info{
            .sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
            .pNext        = nullptr,
            .flags        = 0,
            .bindingCount = binding_count,
            .pBindings    = bindings };
    VKCALL( vkCreateDescriptorSetLayout( g_ctx.device, &info, nullptr, &layout ) );

    return layout;
}

VkDescriptorSet tl::allocate_descriptor_set( VkDescriptorPool pool, VkDescriptorSetLayout layout ) {
    VkDescriptorSet set;

    VkDescriptorSetAllocateInfo info{
            .sType              = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
            .pNext              = nullptr,
            .descriptorPool     = pool,
            .descriptorSetCount = 1,
            .pSetLayouts        = &layout };
    VKCALL( vkAllocateDescriptorSets( g_ctx.device, &info, &set ) );

    return set;
}

void tl::update_descriptor_set( VkDescriptorSet set, const DescriptorWrite* writes, uint32_t write_count ) {
    std::vector<VkWriteDescriptorSet> descriptor_writes( write_count );

    for ( uint32_t i = 0; i < write_count; i++ ) {
        const auto& write = writes[i];

        descriptor_writes[i] = {
                .sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                .pNext           = nullptr,
                .dstSet          = set,
                .dstBinding      = write.binding,
                .dstArrayElement = 0,
                .descriptorCount = 1,
                .descriptorType  = write.type };

        if ( write.type == VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER || write.type == VK_DESCRIPTOR_TYPE_STORAGE_BUFFER ) {
            descriptor_writes[i].pBufferInfo = &write.buffer;
        }
        else {
            descriptor_writes[i].pImageInfo = &write.image;
        }
    }

    vkUpdateDescriptorSets( g_ctx.device, write_count, descriptor_writes.data( ), 0, nullptr );
}


Buffer tl::create_buffer( u64 size, VkBufferUsageFlags usage, VmaAllocationCreateFlags vma_flags, VmaMemoryUsage vma_usage, bool get_device_address, bool map_memory ) {
    assert( size != 0 );

    assert( !map_memory || ( vma_flags & VMA_ALLOCATION_CREATE_MAPPED_BIT ) && "Can't map memory without VMA_ALLOCATION_CREATE_MAPPED_BIT flag" );
    assert( !get_device_address || ( usage & VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT ) && "Can't get device address without VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT flag" );

    Buffer buffer{ .size = size };

    const VkBufferCreateInfo info = {
            .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
            .pNext = nullptr,
            .size  = size,
            .usage = usage };
    const VmaAllocationCreateInfo vma_info = {
            .flags = vma_flags,
            .usage = vma_usage };
    VKCALL( vmaCreateBuffer( g_ctx.allocator, &info, &vma_info, &buffer.buffer, &buffer.allocation, &buffer.allocation_info ) );

    if ( map_memory ) {
        vmaMapMemory( g_ctx.allocator, buffer.allocation, ( void** )&buffer.gpu_data );
        assert( buffer.gpu_data );
    }

    if ( get_device_address ) {
        const VkBufferDeviceAddressInfo address_info = {
                .sType  = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO,
                .pNext  = nullptr,
                .buffer = buffer.buffer };
        buffer.device_address = vkGetBufferDeviceAddress( g_ctx.device, &address_info );
        assert( buffer.device_address );
    }

    return buffer;
}

void tl::destroy_buffer( Buffer& buffer ) {
    if ( buffer.gpu_data ) {
        vmaUnmapMemory( g_ctx.allocator, buffer.allocation );
    }

    vmaDestroyBuffer( g_ctx.allocator, buffer.buffer, buffer.allocation );

    // reset everything
    buffer = { };
}

void tl::upload_buffer_data( const Buffer& buffer, void* data, u64 size, u64 offset ) {
    assert( buffer.gpu_data );
    assert( size != 0 );
    assert( size <= buffer.size );
    assert( data != nullptr );

    memcpy( ( void* )( buffer.gpu_data + offset ), data, size );
}

Image tl::create_image( u32 width, u32 height, VkFormat format, VkImageUsageFlags usage_flags, u32 mip_levels ) {
    Image image;

    image.format = format;
    image.extent = VkExtent3D{ width, height, 1 };

    const VkImageCreateInfo create_info = {
            .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
            .pNext = nullptr,

            .imageType = VK_IMAGE_TYPE_2D,
            .format    = format,
            .extent    = { width, height, 1 },

            .mipLevels   = mip_levels,
            .arrayLayers = 1,

            .samples = VK_SAMPLE_COUNT_1_BIT,
            .tiling  = VK_IMAGE_TILING_OPTIMAL,
            .usage   = usage_flags,
    };

    constexpr VmaAllocationCreateInfo alloc_info = {
            .usage         = VMA_MEMORY_USAGE_GPU_ONLY,
            .requiredFlags = static_cast<VkMemoryPropertyFlags>( VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT ),
    };
    VKCALL( vmaCreateImage( g_ctx.allocator, &create_info, &alloc_info, &image.image, &image.allocation, nullptr ) );

    const VkImageViewCreateInfo view_info{
            .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
            .pNext = nullptr,

            .image            = image.image,
            .viewType         = VK_IMAGE_VIEW_TYPE_2D,
            .format           = image.format,
            .subresourceRange = {
                    .aspectMask     = VkImageAspectFlags( image.format == VK_FORMAT_D32_SFLOAT ? VK_IMAGE_ASPECT_DEPTH_BIT : VK_IMAGE_ASPECT_COLOR_BIT ),
                    .baseMipLevel   = 0,
                    .levelCount     = VK_REMAINING_MIP_LEVELS,
                    .baseArrayLayer = 0,
                    .layerCount     = 1

            } };
    VKCALL( vkCreateImageView( g_ctx.device, &view_info, nullptr, &image.view ) );

    return image;
}

void tl::destroy_image( Image& image ) {
    if ( image.view ) {
        vkDestroyImageView( g_ctx.device, image.view, nullptr );
    }

    vmaDestroyImage( g_ctx.allocator, image.image, image.allocation );
}