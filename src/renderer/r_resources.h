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
#include <vulkan/vulkan_core.h>
#include "pch.h"

namespace tl {
    VkFence                   create_fence( const VkFenceCreateFlags flags = 0 );
    VkSemaphore               create_semaphore( const VkSemaphoreCreateFlags flags = 0 );
    void                      begin_command( VkCommandBuffer cmd, const VkCommandBufferUsageFlags flags = 0 );
    void                      submit_graphics_command( VkCommandBuffer cmd, const VkPipelineStageFlags2 wait_stage, const VkPipelineStageFlags2 signal_stage, const VkSemaphore swapchain_semaphore, const VkSemaphore render_semaphore, const VkFence fence );
    void                      submit_command( VkCommandBuffer cmd, VkQueue queue, const VkFence fence );
    VkRenderingAttachmentInfo attachment( const VkImageView view, const VkClearValue* clear, const VkImageLayout layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL );
    void                      image_barrier( VkCommandBuffer cmd, VkImage image, VkPipelineStageFlags2 src_stage, VkAccessFlags2 src_access, VkImageLayout old_layout, VkPipelineStageFlags2 dst_stage, VkAccessFlags2 dst_access, VkImageLayout new_layout, VkImageAspectFlags aspect_mask = VK_IMAGE_ASPECT_COLOR_BIT, u32 base_mip = 0, u32 level_count = VK_REMAINING_MIP_LEVELS );
    void                      buffer_barrier( VkCommandBuffer cmd, VkBuffer buffer, u64 size, VkPipelineStageFlags2 src_stage, VkAccessFlags2 src_access, VkPipelineStageFlags2 dst_stage, VkAccessFlags2 dst_access );

    // Descriptors
    struct DescriptorWrite {
        VkDescriptorType type;
        uint32_t         binding;
        union {
            VkDescriptorBufferInfo buffer;
            VkDescriptorImageInfo  image;
        };
    };

    VkDescriptorPool      create_descriptor_pool( const VkDescriptorPoolSize* sizes, u32 pool_size, u32 max_sets );
    VkDescriptorSetLayout create_descriptor_layout( const VkDescriptorSetLayoutBinding* bindings, uint32_t binding_count );
    VkDescriptorSet       allocate_descriptor_set( VkDescriptorPool pool, VkDescriptorSetLayout layout );
    void                  update_descriptor_set( VkDescriptorSet set, const DescriptorWrite* writes, uint32_t write_count );

    // Buffers
    struct Buffer {
        VkBuffer          buffer          = VK_NULL_HANDLE;
        u64               size            = 0;
        VmaAllocation     allocation      = { };
        VmaAllocationInfo allocation_info = { };
        VkDeviceAddress   device_address  = 0;
        uintptr_t         gpu_data        = 0;
    };

    Buffer create_buffer( u64 size, VkBufferUsageFlags usage, VmaAllocationCreateFlags vma_flags, VmaMemoryUsage vma_usage, bool get_device_address = false, bool map_memory = false );
    void   destroy_buffer( Buffer& buffer );
    void   upload_buffer_data( const Buffer& buffer, void* data, u64 size, u64 offset = 0 );

    // Image
    struct Image {
        VkImage       image      = VK_NULL_HANDLE;
        VkImageView   view       = VK_NULL_HANDLE;
        VkExtent3D    extent     = { };
        VkFormat      format     = { };
        VmaAllocation allocation = { };
    };
    Image create_image( u32 width, u32 height, VkFormat format, VkImageUsageFlags usage_flags, u32 mip_levels );
    void  destroy_image( Image& image );

} // namespace tl