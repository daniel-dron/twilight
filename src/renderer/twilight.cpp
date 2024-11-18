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

#include "twilight.h"

#include <SDL2/SDL_video.h>
#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <pch.h>
#include "SDL.h"
#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"
#include "glm/ext/matrix_clip_space.hpp"
#include "r_resources.h"
#include "renderer/r_resources.h"
#include "types.h"

#include <renderer/r_context.h>
#include <renderer/r_shaders.h>
#include <vulkan/vulkan_core.h>

using namespace tl;

void Renderer::Initialize( ) {
    // Initialize SDL
    SDL_Init( SDL_INIT_VIDEO );
    SDL_WindowFlags flags = ( SDL_WindowFlags )( SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE );

    m_window = SDL_CreateWindow( "Twilight", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, flags );

    g_ctx.initialize( width, height, "Twilight", m_window );

    m_camera.near        = 0.001f;
    m_camera.far         = 1000.0f;
    m_camera.fov         = 70.0f;
    m_camera.position    = glm::vec3( 0.0f, 0.0f, 20.0f );
    m_camera.orientation = glm::quat( 0.0f, 0.0f, 0.0f, 1.0f );

    m_pipeline.initialize( PipelineConfig{
            .name                 = "mesh",
            .vertex               = "../shaders/mesh.vert.spv",
            .pixel                = "../shaders/mesh.frag.spv",
            .cull_mode            = VK_CULL_MODE_NONE,
            .color_targets        = { PipelineConfig::ColorTargetsConfig{ .format = g_ctx.swapchain.format, .blend_type = PipelineConfig::BlendType::OFF } },
            .push_constant_ranges = { VkPushConstantRange{ .stageFlags = VK_SHADER_STAGE_ALL_GRAPHICS, .size = sizeof( ScenePushConstants ) } },
    } );

    // create global staging buffer (100MB)
    m_staging_buffer = create_buffer( 1000 * 1000 * 100, VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, VMA_ALLOCATION_CREATE_MAPPED_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU, false, true );

    m_mesh               = load_mesh_from_file( "../../assets/teapot/teapot.gltf", "Teapot" ).value( );
    m_mesh.vertex_buffer = create_buffer( m_mesh.vertices.size( ) * sizeof( Vertex ), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true, false );
    m_mesh.index_buffer  = create_buffer( m_mesh.indices.size( ) * sizeof( u32 ), VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY );

    // upload mesh
    upload_buffer_data( m_staging_buffer, m_mesh.vertices.data( ), m_mesh.vertex_buffer.size );
    upload_buffer_data( m_staging_buffer, m_mesh.indices.data( ), m_mesh.index_buffer.size, m_mesh.vertex_buffer.size ); // Upload to the same staging buffer after the vertex data;

    {
        auto& cmd = g_ctx.global_cmd;
        VKCALL( vkResetFences( g_ctx.device, 1, &g_ctx.global_fence ) );
        VKCALL( vkResetCommandBuffer( cmd, 0 ) );
        begin_command( cmd, VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT );

        const VkBufferCopy vertex_copy = {
                .srcOffset = 0,
                .dstOffset = 0,
                .size      = m_mesh.vertex_buffer.size };
        const VkBufferCopy index_copy = {
                .srcOffset = m_mesh.vertex_buffer.size,
                .dstOffset = 0,
                .size      = m_mesh.index_buffer.size };
        vkCmdCopyBuffer( cmd, m_staging_buffer.buffer, m_mesh.vertex_buffer.buffer, 1, &vertex_copy );
        vkCmdCopyBuffer( cmd, m_staging_buffer.buffer, m_mesh.index_buffer.buffer, 1, &index_copy );

        VKCALL( vkEndCommandBuffer( cmd ) );
        submit_command( cmd, g_ctx.graphics_queue, g_ctx.global_fence );
        VKCALL( vkWaitForFences( g_ctx.device, 1, &g_ctx.global_fence, true, UINT64_MAX ) );
    }
}

void Renderer::Shutdown( ) {
    vkDeviceWaitIdle( g_ctx.device );

    destroy_buffer( m_staging_buffer );
    destroy_buffer( m_mesh.vertex_buffer );
    destroy_buffer( m_mesh.index_buffer );

    m_pipeline.shutdown( );
    g_ctx.shutdown( );
    SDL_DestroyWindow( m_window );
}

void Renderer::Run( ) {
    while ( !m_quit ) {
        process_events( );

        auto& frame               = g_ctx.get_current_frame( );
        auto  cmd                 = frame.cmd;
        u32   swapchain_image_idx = 0;

        // Synchronization
        // Wait for the fence of the frame (will sync CPU-GPU)
        // Acquire next image after this frame was already presented to the swapchain
        VKCALL( vkWaitForFences( g_ctx.device, 1, &frame.fence, true, UINT64_MAX ) );
        VKCALL( vkResetFences( g_ctx.device, 1, &frame.fence ) );
        VKCALL( vkAcquireNextImageKHR( g_ctx.device, g_ctx.swapchain.swapchain, UINT64_MAX, frame.swapchain_semaphore, nullptr, &swapchain_image_idx ) );

        // Start command
        VKCALL( vkResetCommandBuffer( cmd, 0 ) );
        begin_command( cmd, VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT );

        image_barrier( cmd, g_ctx.swapchain.images[swapchain_image_idx],
                       VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED,
                       VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT_KHR, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL );
        tick( swapchain_image_idx );
        image_barrier( cmd, g_ctx.swapchain.images[swapchain_image_idx],
                       VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT_KHR, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                       VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT_KHR, VK_ACCESS_2_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR );

        // Submit command
        VKCALL( vkEndCommandBuffer( cmd ) );
        submit_graphics_command( cmd, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT_KHR, VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT, frame.swapchain_semaphore, frame.render_semaphore, frame.fence );

        // Present
        const VkPresentInfoKHR present_info = {
                .sType              = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR,
                .pNext              = nullptr,
                .waitSemaphoreCount = 1,
                .pWaitSemaphores    = &frame.render_semaphore,
                .swapchainCount     = 1,
                .pSwapchains        = &g_ctx.swapchain.swapchain,
                .pImageIndices      = &swapchain_image_idx };
        VKCALL( vkQueuePresentKHR( g_ctx.graphics_queue, &present_info ) );

        g_ctx.current_frame++;
    }
}

void Renderer::process_events( ) {
    SDL_Event event;
    while ( SDL_PollEvent( &event ) != 0 ) {
        if ( event.type == SDL_QUIT ) {
            m_quit = true;
        }

        if ( event.type == SDL_WINDOWEVENT ) {
            if ( event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED && event.window.data1 > 0 && event.window.data2 > 0 ) {
                this->width  = event.window.data1;
                this->height = event.window.data2;
                g_ctx.swapchain.resize( event.window.data1, event.window.data2, g_ctx.chosen_gpu, g_ctx.device, g_ctx.surface );
            }
        }
    }
}

void Renderer::tick( u32 swapchain_image_idx ) {
    auto& frame = g_ctx.get_current_frame( );
    auto& cmd   = frame.cmd;

    VkClearValue    clear_color{ 0.1f, 0.05f, 0.1f, 1.0f };
    std::array      attachments = { attachment( g_ctx.swapchain.views[swapchain_image_idx], &clear_color ) };
    VkRenderingInfo render_info = {
            .sType                = VK_STRUCTURE_TYPE_RENDERING_INFO,
            .pNext                = nullptr,
            .renderArea           = VkRect2D{ VkOffset2D{ 0, 0 }, VkExtent2D{ width, height } },
            .layerCount           = 1,
            .colorAttachmentCount = ( u32 )attachments.size( ),
            .pColorAttachments    = attachments.data( ) };
    vkCmdBeginRendering( cmd, &render_info );

    VkViewport viewport = {
            .x = 0, .y = 0, .width = ( float )width, .height = ( float )height, .minDepth = 0.0f, .maxDepth = 1.0f };
    vkCmdSetViewport( cmd, 0, 1, &viewport );

    const VkRect2D scissor = {
            .offset = { .x = 0, .y = 0 },
            .extent = { .width = width, .height = height } };
    vkCmdSetScissor( cmd, 0, 1, &scissor );

    vkCmdBindPipeline( cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipeline.pipeline );

    // Camera matrices
    glm::mat4 view       = glm::mat4_cast( m_camera.orientation );
    view[3]              = glm::vec4( m_camera.position, 1.0f );
    view                 = inverse( view );
    glm::mat4 projection = glm::perspective( m_camera.fov, ( float )width / ( float )height, m_camera.near, m_camera.far );

    ScenePushConstants pc{
            .view          = view,
            .projection    = projection,
            .vertex_buffer = m_mesh.vertex_buffer.device_address };
    vkCmdPushConstants( cmd, m_pipeline.layout, VK_SHADER_STAGE_ALL_GRAPHICS, 0, sizeof( ScenePushConstants ), &pc );

    vkCmdBindIndexBuffer( cmd, m_mesh.index_buffer.buffer, 0, VK_INDEX_TYPE_UINT32 );
    vkCmdDrawIndexed( cmd, ( u32 )m_mesh.indices.size( ), 1, 0, 0, 0 );

    vkCmdEndRendering( cmd );
}

std::optional<Mesh> tl::load_mesh_from_file( const std::string& gltf_path, const std::string& mesh_name ) {
    assert( !gltf_path.empty( ) );
    assert( !mesh_name.empty( ) );
    assert( std::filesystem::exists( gltf_path ) );

    Assimp::Importer importer;
    const auto       aiScene = importer.ReadFile( gltf_path, aiProcess_Triangulate );

    for ( size_t i = 0; i < aiScene->mNumMeshes; i++ ) {
        if ( std::string( aiScene->mMeshes[i]->mName.C_Str( ) ) == mesh_name ) {
            Mesh mesh;
            mesh.vertices.clear( );
            mesh.indices.clear( );

            auto ai_mesh = aiScene->mMeshes[i];

            mesh.vertices.reserve( ai_mesh->mNumVertices );
            for ( u32 i = 0; i < ai_mesh->mNumVertices; i++ ) {
                auto vertex = ai_mesh->mVertices[i];
                mesh.vertices.push_back( { vertex.x, vertex.y, vertex.z } );
            }

            mesh.indices.reserve( ai_mesh->mNumFaces * 3 );
            for ( u32 i = 0; i < ai_mesh->mNumFaces; i++ ) {
                auto& face = ai_mesh->mFaces[i];
                mesh.indices.emplace_back( face.mIndices[0] );
                mesh.indices.emplace_back( face.mIndices[1] );
                mesh.indices.emplace_back( face.mIndices[2] );
            }

            return mesh;
        }
    }

    return std::nullopt;
}