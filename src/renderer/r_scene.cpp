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

#include <optional>
#include <pch.h>
#include <stdexcept>
#include <vulkan/vulkan_core.h>
#include "renderer/r_context.h"
#include "renderer/r_resources.h"
#include "renderer/r_shaders.h"

#include "r_scene.h"

void tl::GPUScene::initialize( ) {
    // Create free list
    m_free_spots.reserve( m_renderables_max );
    for ( i32 i = m_renderables_max - 1; i >= 0; i-- ) {
        m_free_spots.push_back( { u32( i ) } );
    }

    m_renderables.resize( m_renderables_max, SubMeshRenderable{ glm::mat4( 1.0f ), 0, R_INVALID } );

    m_renderables_gpu       = create_buffer( sizeof( SubMeshRenderable ) * m_renderables_max, VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true );
    m_dirty_renderables_gpu = create_buffer( sizeof( SubMeshRenderableUpdate ) * m_renderables_max, VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_ALLOCATION_CREATE_MAPPED_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU, true, true );

    m_copy_pipeline.initialize( PipelineConfig{
            .name                 = "update renderables",
            .compute              = "../shaders/cs_dirty_update.comp.slang.spv",
            .push_constant_ranges = { VkPushConstantRange{ .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT, .size = sizeof( UpdateRenderablesPushConstants ) } } } );
}

void tl::GPUScene::shutdown( ) {
    destroy_buffer( m_renderables_gpu );
    destroy_buffer( m_dirty_renderables_gpu );
    m_copy_pipeline.shutdown( );
}

Handle<tl::SubMeshRenderable> tl::GPUScene::register_renderable( SubMeshRenderable* renderable ) {
    if ( auto handle = get_free_handle( ); handle.has_value( ) ) {
        renderable->flags |= R_VALID;

        m_renderables[handle->handle] = *renderable;
        m_dirty_renderables.push_back( SubMeshRenderableUpdate{ *handle, *renderable } );

        return *handle;
    }

    throw std::runtime_error( "No more valid handles!" );
}

const tl::SubMeshRenderable& tl::GPUScene::get_object( Handle<SubMeshRenderable> handle ) const {
    return m_renderables.at( handle.handle );
}

void tl::GPUScene::update_transform( Handle<SubMeshRenderable> handle, const glm::mat4& transform ) {
    auto& object     = m_renderables.at( handle.handle );
    object.transform = transform;

    m_dirty_renderables.push_back( SubMeshRenderableUpdate{ .handle = handle, .new_value = object } );
}

std::optional<Handle<tl::SubMeshRenderable>> tl::GPUScene::get_free_handle( ) {
    if ( m_free_spots.empty( ) ) {
        return std::nullopt;
    }

    auto handle = m_free_spots.back( );
    m_free_spots.pop_back( );
    return handle;
}

void tl::GPUScene::sync( ) {

    // if 80%+ of the renderable were updated this frame, just update the entire scene
    if ( m_dirty_renderables.size( ) >= m_renderables_max * 0.8f ) {
        auto& frame = g_ctx.get_current_frame( );
        auto& cmd   = frame.cmd;
        
        upload_buffer_data( g_ctx.staging_buffer, m_renderables.data( ), sizeof( SubMeshRenderable ) * m_renderables_max );

        const VkBufferCopy draws_copy = {
                .srcOffset = 0,
                .dstOffset = 0,
                .size      = sizeof( SubMeshRenderable ) * m_renderables_max };
        vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, m_renderables_gpu.buffer, 1, &draws_copy );

        m_dirty_renderables.clear( );
    }
    else if ( m_dirty_renderables.size( ) > 0 ) {
        auto& frame = g_ctx.get_current_frame( );
        auto& cmd   = frame.cmd;

        upload_buffer_data( m_dirty_renderables_gpu, m_dirty_renderables.data( ), m_dirty_renderables.size( ) * sizeof( SubMeshRenderableUpdate ) );

        vkCmdBindPipeline( cmd, VK_PIPELINE_BIND_POINT_COMPUTE, m_copy_pipeline.pipeline );

        buffer_barrier( cmd, m_renderables_gpu.buffer, m_renderables_gpu.size, VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT, VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT );
        buffer_barrier( cmd, m_dirty_renderables_gpu.buffer, m_dirty_renderables_gpu.size, VK_PIPELINE_STAGE_2_HOST_BIT, VK_ACCESS_2_HOST_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT );

        UpdateRenderablesPushConstants pc = {
                .renderables = m_renderables_gpu.device_address,
                .updates     = m_dirty_renderables_gpu.device_address,
                .count       = m_dirty_renderables.size( ) };
        vkCmdPushConstants( cmd, m_copy_pipeline.layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( UpdateRenderablesPushConstants ), &pc );
        vkCmdDispatch( cmd, u32( m_dirty_renderables.size( ) + 31 ) / 32, 1, 1 );

        buffer_barrier( cmd, m_renderables_gpu.buffer, m_renderables_gpu.size, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT, VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT );

        m_dirty_renderables.clear( );
    }
}