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

namespace tl {
    VkFence                   create_fence( const VkFenceCreateFlags flags = 0 );
    VkSemaphore               create_semaphore( const VkSemaphoreCreateFlags flags = 0 );
    void                      begin_command( VkCommandBuffer cmd, const VkCommandBufferUsageFlags flags = 0 );
    void                      submit_command( VkCommandBuffer cmd, const VkPipelineStageFlags2 wait_stage, const VkPipelineStageFlags2 signal_stage, const VkSemaphore swapchain_semaphore, const VkSemaphore render_semaphore, const VkFence fence );
    VkRenderingAttachmentInfo attachment( const VkImageView view, const VkClearValue* clear, const VkImageLayout layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL );
} // namespace tl