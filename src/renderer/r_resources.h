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

namespace tl {
    VkFence                   create_fence( const VkFenceCreateFlags flags = 0 );
    VkSemaphore               create_semaphore( const VkSemaphoreCreateFlags flags = 0 );
    void                      begin_command( VkCommandBuffer cmd, const VkCommandBufferUsageFlags flags = 0 );
    void                      submit_command( VkCommandBuffer cmd, const VkPipelineStageFlags2 wait_stage, const VkPipelineStageFlags2 signal_stage, const VkSemaphore swapchain_semaphore, const VkSemaphore render_semaphore, const VkFence fence );
    VkRenderingAttachmentInfo attachment( const VkImageView view, const VkClearValue* clear, const VkImageLayout layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL );
    void                      image_barrier( VkCommandBuffer cmd, VkImage image, VkPipelineStageFlags2 src_stage, VkAccessFlags2 src_access, VkImageLayout old_layout, VkPipelineStageFlags2 dst_stage, VkAccessFlags2 dst_access, VkImageLayout new_layout, VkImageAspectFlags aspect_mask = VK_IMAGE_ASPECT_COLOR_BIT, u32 base_mip = 0, u32 level_count = VK_REMAINING_MIP_LEVELS );

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


} // namespace tl