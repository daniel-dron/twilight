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
    const VkCommandBufferSubmitInfo buffer_info = {
            .sType         = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO,
            .pNext         = nullptr,
            .commandBuffer = cmd,
            .deviceMask    = 0,
    };

    const VkSemaphoreSubmitInfo wait_info = {
            .sType       = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
            .pNext       = nullptr,
            .semaphore   = wait_semaphore,
            .value       = 1,
            .stageMask   = wait_stage,
            .deviceIndex = 0 };

    const VkSemaphoreSubmitInfo signal_info = {
            .sType       = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
            .pNext       = nullptr,
            .semaphore   = render_semaphore,
            .value       = 1,
            .stageMask   = signal_stage,
            .deviceIndex = 0 };

    const VkSubmitInfo2 info = {
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
    const VkRenderingAttachmentInfo color_attachment = {
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