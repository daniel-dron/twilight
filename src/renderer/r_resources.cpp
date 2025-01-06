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
#include <cstddef>
#include <optional>
#include <pch.h>
#include <vulkan/vulkan_core.h>

#include <nlohmann/json.hpp>
#include <utils.h>
#include "r_context.h"
#include "r_resources.h"
#include "renderer/r_context.h"
#include "types.h"

using namespace tl;

VkFence tl::create_fence( const char* name, const VkFenceCreateFlags flags ) {
    VkFence fence;

    const VkFenceCreateInfo info = {
            .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
            .pNext = nullptr,
            .flags = flags,
    };

    VKCALL( vkCreateFence( g_ctx.device, &info, nullptr, &fence ) );

    set_object_name( g_ctx.device, VK_OBJECT_TYPE_FENCE, reinterpret_cast<u64>( fence ), name );

    return fence;
}

VkSemaphore tl::create_semaphore( const char* name, const VkSemaphoreCreateFlags flags ) {
    VkSemaphore semaphore;

    const VkSemaphoreCreateInfo info{
            .sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO,
            .pNext = nullptr,
            .flags = flags };

    VKCALL( vkCreateSemaphore( g_ctx.device, &info, nullptr, &semaphore ) );

    set_object_name( g_ctx.device, VK_OBJECT_TYPE_SEMAPHORE, reinterpret_cast<u64>( semaphore ), name );

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
                    .baseMipLevel   = base_mip,
                    .levelCount     = level_count,
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

namespace {
    VkDescriptorType get_descriptor_type( const nlohmann::json& type ) {
        const auto& kind = type["kind"].get<std::string>( );

        if ( kind == "resource" ) {
            const auto& shape = type["baseShape"].get<std::string>( );
            const bool  is_rw = type.contains( "access" ) && type["access"] == "readWrite";

            if ( shape == "texture2D" ) {
                return is_rw ? VK_DESCRIPTOR_TYPE_STORAGE_IMAGE : VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
            }

            // ...
        }
        else if ( kind == "samplerState" ) {
            return VK_DESCRIPTOR_TYPE_SAMPLER;
        }
        else if ( kind == "array" ) {
            return get_descriptor_type( type["elementType"] );
        }

        return VK_DESCRIPTOR_TYPE_MAX_ENUM;
    }
} // namespace

ShaderBindings tl::parse_shader_bindings( const std::string& json_content ) {
    ShaderBindings result;
    nlohmann::json shader_json = nlohmann::json::parse( json_content );

    std::unordered_map<uint32_t, VkDescriptorSetLayoutBinding> binding_map;

    // Process parameters
    for ( const auto& param : shader_json["parameters"] ) {
        const auto&       binding_info = param["binding"];
        const std::string binding_kind = binding_info["kind"].get<std::string>( );

        if ( binding_kind == "descriptorTableSlot" ) {
            uint32_t binding_index = binding_info["index"].get<uint32_t>( );

            // Skip if we already processed this binding
            if ( binding_map.find( binding_index ) != binding_map.end( ) ) {
                continue;
            }

            VkDescriptorSetLayoutBinding layout_binding = { };
            layout_binding.binding                      = binding_index;
            layout_binding.descriptorType               = get_descriptor_type( param["type"] );
            layout_binding.descriptorCount              = 1;
            layout_binding.stageFlags                   = VK_SHADER_STAGE_COMPUTE_BIT; // Assuming compute shader for now

            // Handle arrays
            if ( param["type"]["kind"] == "array" ) {
                if ( param["type"]["elementCount"].is_number( ) ) {
                    layout_binding.descriptorCount = param["type"]["elementCount"].get<uint32_t>( );
                }
                else {
                    // If elementCount is 0 or not specified, we'll use a reasonable default
                    layout_binding.descriptorCount = 16;
                }
            }

            binding_map[binding_index] = layout_binding;
        }
        else if ( binding_kind == "pushConstantBuffer" ) {
            if ( param["type"]["elementType"].contains( "size" ) ) {
                result.push_constant_size = param["type"]["elementType"]["binding"]["size"].get<size_t>( );
            }
        }
    }

    result.descriptor_bindings.reserve( binding_map.size( ) );
    for ( const auto& [_, binding] : binding_map ) {
        result.descriptor_bindings.push_back( binding );
    }

    return result;
}

std::optional<ShaderBindings> tl::create_shader_descriptor_layout( const std::string& json_path ) {
    auto           json_content = read_file_to_string( json_path.c_str( ) );
    ShaderBindings bindings     = parse_shader_bindings( json_content );

    if ( bindings.descriptor_bindings.empty( ) ) {
        return std::nullopt;
    }

    bindings.layout = create_descriptor_layout(
            bindings.descriptor_bindings.data( ),
            static_cast<uint32_t>( bindings.descriptor_bindings.size( ) ) );

    return bindings;
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

std::vector<VkDescriptorSet> tl::allocate_descript_set( VkDescriptorPool pool, VkDescriptorSetLayout layout, u32 count ) {
    std::vector<VkDescriptorSet> sets;

    for ( auto i = 0; i < count; i++ ) {
        sets.push_back( allocate_descriptor_set( pool, layout ) );
    }

    return sets;
}


VkSampler tl::create_sampler( VkSamplerCreateInfo info ) {
    VkSampler sampler;
    vkCreateSampler( g_ctx.device, &info, nullptr, &sampler );

    return sampler;
}

VkSampler tl::create_reduction_sampler( ) {
    VkSamplerCreateInfo info{
            .sType        = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO,
            .magFilter    = VK_FILTER_LINEAR,
            .minFilter    = VK_FILTER_LINEAR,
            .mipmapMode   = VK_SAMPLER_MIPMAP_MODE_NEAREST,
            .addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
            .addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
            .addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
            .minLod       = 0,
            .maxLod       = 16.f,
    };

    VkSamplerReductionModeCreateInfoEXT info_reduction{
            .sType         = VK_STRUCTURE_TYPE_SAMPLER_REDUCTION_MODE_CREATE_INFO_EXT,
            .pNext         = nullptr,
            .reductionMode = VK_SAMPLER_REDUCTION_MODE_MIN,
    };

    info.pNext = &info_reduction;

    return create_sampler( info );
}

Buffer tl::create_buffer( const char* name, u64 size, VkBufferUsageFlags usage, VmaAllocationCreateFlags vma_flags, VmaMemoryUsage vma_usage, bool get_device_address, bool map_memory ) {
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

    set_object_name( g_ctx.device, VK_OBJECT_TYPE_BUFFER, reinterpret_cast<u64>( buffer.buffer ), name );

    return buffer;
}

FBuffer tl::create_fbuffer( const char* name, u64 size, VkBufferUsageFlags usage, VmaAllocationCreateFlags vma_flags, VmaMemoryUsage vma_usage, bool get_device_address, bool map_memory ) {
    FBuffer fbuffer = { };

    for ( u32 i = 0; i < g_ctx.frame_overlap; i++ ) {
        fbuffer.buffers.push_back( create_buffer( name, size, usage, vma_flags, vma_usage, get_device_address, map_memory ) );
    }

    return fbuffer;
}

Buffer& FBuffer::get( ) {
    return buffers.at( g_ctx.get_current_frame_index( ) );
}

void tl::destroy_buffer( Buffer& buffer ) {
    if ( buffer.gpu_data ) {
        vmaUnmapMemory( g_ctx.allocator, buffer.allocation );
    }

    vmaDestroyBuffer( g_ctx.allocator, buffer.buffer, buffer.allocation );

    // reset everything
    buffer = { };
}

void tl::destroy_fbuffer( FBuffer& buffer ) {
    for ( auto& b : buffer.buffers ) {
        destroy_buffer( b );
    }
}

void tl::upload_buffer_data( const Buffer& buffer, void* data, u64 size, u64 offset ) {
    assert( buffer.gpu_data );
    assert( size != 0 );
    assert( size <= buffer.size );
    assert( data != nullptr );

    memcpy( ( void* )( buffer.gpu_data + offset ), data, size );
}

Image tl::create_image( const char* name, u32 width, u32 height, VkFormat format, VkImageUsageFlags usage_flags, u32 mip_levels ) {
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

    set_object_name( g_ctx.device, VK_OBJECT_TYPE_IMAGE, reinterpret_cast<u64>( image.image ), name );

    return image;
}

VkImageView tl::create_view( const char* name, const Image& image, u32 mip, u32 mip_count ) {
    VkImageView view = VK_NULL_HANDLE;

    const VkImageViewCreateInfo view_info{
            .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
            .pNext = nullptr,

            .image            = image.image,
            .viewType         = VK_IMAGE_VIEW_TYPE_2D,
            .format           = image.format,
            .subresourceRange = {
                    .aspectMask     = VkImageAspectFlags( image.format == VK_FORMAT_D32_SFLOAT ? VK_IMAGE_ASPECT_DEPTH_BIT : VK_IMAGE_ASPECT_COLOR_BIT ),
                    .baseMipLevel   = mip,
                    .levelCount     = mip_count,
                    .baseArrayLayer = 0,
                    .layerCount     = 1

            } };
    VKCALL( vkCreateImageView( g_ctx.device, &view_info, nullptr, &view ) );

    set_object_name( g_ctx.device, VK_OBJECT_TYPE_IMAGE_VIEW, reinterpret_cast<u64>( view ), name );

    return view;
}

void tl::destroy_image( Image& image ) {
    if ( image.view ) {
        vkDestroyImageView( g_ctx.device, image.view, nullptr );
    }

    vmaDestroyImage( g_ctx.allocator, image.image, image.allocation );
}

u32 tl::get_mip_count( u32 width, u32 height ) {
    u32 count = 1;

    while ( width > 1 || height > 1 ) {
        count++;

        width /= 2;
        height /= 2;
    }

    return count;
}

// Debug
void tl::begin_debug_region( VkCommandBuffer cmd, const char* name, const glm::vec4& color ) {
    VkDebugUtilsLabelEXT label = {
            .sType      = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT,
            .pLabelName = name,
            .color      = { color.x, color.y, color.z, color.w },
    };
    vkCmdBeginDebugUtilsLabelEXT( cmd, &label );
}

void tl::end_debug_region( VkCommandBuffer cmd ) {
    vkCmdEndDebugUtilsLabelEXT( cmd );
}

void tl::insert_debug_label( VkCommandBuffer cmd, const char* name, const glm::vec4& color ) {
    VkDebugUtilsLabelEXT label = {
            .sType      = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT,
            .pLabelName = name,
            .color      = { color.x, color.y, color.z, color.w },
    };
    vkCmdInsertDebugUtilsLabelEXT( cmd, &label );
}

void tl::set_object_name( VkDevice device, VkObjectType object_type, uint64_t handle, const char* name ) {
    VkDebugUtilsObjectNameInfoEXT name_info = {
            .sType        = VK_STRUCTURE_TYPE_DEBUG_UTILS_OBJECT_NAME_INFO_EXT,
            .objectType   = object_type,
            .objectHandle = handle,
            .pObjectName  = name,
    };
    vkSetDebugUtilsObjectNameEXT( device, &name_info );
}

tl::DebugRegion::DebugRegion( VkCommandBuffer cmd, const char* name, const glm::vec4& color ) {
    m_cmd = cmd;
    begin_debug_region( cmd, name, color );
}

tl::DebugRegion::~DebugRegion( ) {
    end_debug_region( m_cmd );
}