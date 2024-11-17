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

void tl::submit_command( VkCommandBuffer cmd, const VkPipelineStageFlags2 wait_stage, const VkPipelineStageFlags2 signal_stage, const VkSemaphore wait_semaphore, const VkSemaphore render_semaphore, const VkFence fence ) {
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

        if ( write.type == VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER ||
             write.type == VK_DESCRIPTOR_TYPE_STORAGE_BUFFER ) {
            descriptor_writes[i].pBufferInfo = &write.buffer;
        }
        else {
            descriptor_writes[i].pImageInfo = &write.image;
        }
    }

    vkUpdateDescriptorSets( g_ctx.device, write_count, descriptor_writes.data( ), 0, nullptr );
}