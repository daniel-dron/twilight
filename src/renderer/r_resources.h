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
#include <vector>
#include <vulkan/vulkan_core.h>
#include "pch.h"

namespace tl {
    VkFence                   create_fence( const char* name, const VkFenceCreateFlags flags = 0 );
    VkSemaphore               create_semaphore( const char* name, const VkSemaphoreCreateFlags flags = 0 );
    void                      begin_command( VkCommandBuffer cmd, const VkCommandBufferUsageFlags flags = 0 );
    void                      submit_graphics_command( VkCommandBuffer cmd, const VkPipelineStageFlags2 wait_stage, const VkPipelineStageFlags2 signal_stage, const VkSemaphore swapchain_semaphore, const VkSemaphore render_semaphore, const VkFence fence );
    void                      submit_command( VkCommandBuffer cmd, VkQueue queue, const VkFence fence );
    VkRenderingAttachmentInfo attachment( const VkImageView view, const VkClearValue* clear, const VkImageLayout layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL );
    void                      image_barrier( VkCommandBuffer cmd, VkImage image, VkPipelineStageFlags2 src_stage, VkAccessFlags2 src_access, VkImageLayout old_layout, VkPipelineStageFlags2 dst_stage, VkAccessFlags2 dst_access, VkImageLayout new_layout, VkImageAspectFlags aspect_mask = VK_IMAGE_ASPECT_COLOR_BIT, u32 base_mip = 0, u32 level_count = VK_REMAINING_MIP_LEVELS );
    void                      buffer_barrier( VkCommandBuffer cmd, VkBuffer buffer, u64 size, VkPipelineStageFlags2 src_stage, VkAccessFlags2 src_access, VkPipelineStageFlags2 dst_stage, VkAccessFlags2 dst_access );

    // Descriptors
    struct ShaderBindings {
        std::vector<VkDescriptorSetLayoutBinding> descriptor_bindings;
        size_t                                    push_constant_size = 0;
        VkDescriptorSetLayout                     layout;
    };

    struct DescriptorWrite {
        VkDescriptorType type;
        uint32_t         binding;
        union {
            VkDescriptorBufferInfo buffer;
            VkDescriptorImageInfo  image;
        };
    };

    ShaderBindings                parse_shader_bindings( const std::string& json_content );
    std::optional<ShaderBindings> create_shader_descriptor_layout( const std::string& json_path );

    VkDescriptorPool             create_descriptor_pool( const VkDescriptorPoolSize* sizes, u32 pool_size, u32 max_sets );
    VkDescriptorSetLayout        create_descriptor_layout( const VkDescriptorSetLayoutBinding* bindings, uint32_t binding_count );
    VkDescriptorSet              allocate_descriptor_set( VkDescriptorPool pool, VkDescriptorSetLayout layout );
    std::vector<VkDescriptorSet> allocate_descript_set( VkDescriptorPool pool, VkDescriptorSetLayout layout, u32 count );
    void                         update_descriptor_set( VkDescriptorSet set, const DescriptorWrite* writes, uint32_t write_count );

    VkSampler create_sampler( VkSamplerCreateInfo info = {
                                      .sType                   = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO,
                                      .magFilter               = VK_FILTER_LINEAR,
                                      .minFilter               = VK_FILTER_LINEAR,
                                      .mipmapMode              = VK_SAMPLER_MIPMAP_MODE_LINEAR,
                                      .addressModeU            = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
                                      .addressModeV            = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
                                      .addressModeW            = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
                                      .anisotropyEnable        = VK_TRUE,
                                      .maxAnisotropy           = 16.0f,
                                      .compareEnable           = VK_FALSE,
                                      .compareOp               = VK_COMPARE_OP_ALWAYS,
                                      .borderColor             = VK_BORDER_COLOR_INT_OPAQUE_BLACK,
                                      .unnormalizedCoordinates = VK_FALSE,
                              } );
    VkSampler create_reduction_sampler( );

    // Buffers
    struct Buffer {
        VkBuffer          buffer          = VK_NULL_HANDLE;
        u64               size            = 0;
        VmaAllocation     allocation      = { };
        VmaAllocationInfo allocation_info = { };
        VkDeviceAddress   device_address  = 0;
        uintptr_t         gpu_data        = 0;
    };

    struct FBuffer {
        std::vector<Buffer> buffers;

        Buffer& get( );
    };

    Buffer  create_buffer( const char* name, u64 size, VkBufferUsageFlags usage, VmaAllocationCreateFlags vma_flags, VmaMemoryUsage vma_usage, bool get_device_address = false, bool map_memory = false );
    FBuffer create_fbuffer( const char* name, u64 size, VkBufferUsageFlags usage, VmaAllocationCreateFlags vma_flags, VmaMemoryUsage vma_usage, bool get_device_address = false, bool map_memory = false );
    void    destroy_buffer( Buffer& buffer );
    void    destroy_fbuffer( FBuffer& buffer );
    void    upload_buffer_data( const Buffer& buffer, void* data, u64 size, u64 offset = 0 );

    template<typename T>
    T read_from_buffer( Buffer buffer, u64 offset = 0 ) {
        return *( T* )( buffer.gpu_data + offset );
    }

    // Image
    struct Image {
        VkImage       image      = VK_NULL_HANDLE;
        VkImageView   view       = VK_NULL_HANDLE;
        VkExtent3D    extent     = { };
        VkFormat      format     = { };
        VmaAllocation allocation = { };
    };
    Image       create_image( const char* name, u32 width, u32 height, VkFormat format, VkImageUsageFlags usage_flags, u32 mip_levels );
    VkImageView create_view( const char* name, const Image& image, u32 mip = 0, u32 mip_count = VK_REMAINING_MIP_LEVELS );
    void        destroy_image( Image& image );

    u32 get_mip_count( u32 width, u32 height );

    // Debug
    void begin_debug_region( VkCommandBuffer cmd, const char* name, const glm::vec4& color );
    void end_debug_region( VkCommandBuffer cmd );
    void insert_debug_label( VkCommandBuffer cmd, const char* name, const glm::vec4& color );
    void set_object_name( VkDevice device, VkObjectType object_type, uint64_t handle, const char* name );

#define DEBUG_REGION( cmd, name ) DebugRegion debug_region##__LINE__( cmd, name, comptime_color( name ) )

    class DebugRegion {
    public:
        DebugRegion( VkCommandBuffer cmd, const char* name, const glm::vec4& color );
        ~DebugRegion( );

        // Delete copy constructor and assignment operator to prevent misuse
        DebugRegion( const DebugRegion& )            = delete;
        DebugRegion& operator=( const DebugRegion& ) = delete;

    private:
        VkCommandBuffer m_cmd;
    };
} // namespace tl