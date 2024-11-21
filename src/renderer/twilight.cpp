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
#include <format>
#include <meshoptimizer.h>
#include <optional>
#include <pch.h>
#include "SDL.h"
#include "SDL_events.h"
#include "SDL_scancode.h"
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
    m_camera.far         = 5000.0f;
    m_camera.fov         = 70.0f;
    m_camera.position    = glm::vec3( 600.0f, 500.0f, 1000.0f );
    // m_camera.position    = glm::vec3( 0.0f, 0.0f, 10.0f );
    m_camera.orientation = glm::quat( 0.0f, 0.0f, 0.0f, 1.0f );

    m_mesh_pipeline.initialize( PipelineConfig{
            .name                 = "mesh",
            .pixel                = "../shaders/mesh.frag.spv",
            .mesh                 = "../shaders/mesh.mesh.spv",
            .task                 = "../shaders/mesh.task.spv",
            .cull_mode            = VK_CULL_MODE_BACK_BIT,
            .front_face           = VK_FRONT_FACE_CLOCKWISE,
            .color_targets        = { PipelineConfig::ColorTargetsConfig{ .format = g_ctx.swapchain.format, .blend_type = PipelineConfig::BlendType::OFF } },
            .push_constant_ranges = { VkPushConstantRange{ .stageFlags = VK_SHADER_STAGE_ALL_GRAPHICS | VK_SHADER_STAGE_MESH_BIT_EXT | VK_SHADER_STAGE_TASK_BIT_EXT, .size = sizeof( ScenePushConstants ) } },
    } );
    m_pipeline.initialize( PipelineConfig{
            .name                 = "mesh",
            .vertex               = "../shaders/mesh.vert.spv",
            .pixel                = "../shaders/mesh.frag.spv",
            .cull_mode            = VK_CULL_MODE_BACK_BIT,
            .front_face           = VK_FRONT_FACE_CLOCKWISE,
            .color_targets        = { PipelineConfig::ColorTargetsConfig{ .format = g_ctx.swapchain.format, .blend_type = PipelineConfig::BlendType::OFF } },
            .push_constant_ranges = { VkPushConstantRange{ .stageFlags = VK_SHADER_STAGE_ALL_GRAPHICS, .size = sizeof( ScenePushConstants ) } },
    } );

    // m_mesh               = load_mesh_from_file( "../../assets/teapot/teapot.gltf", "Teapot" ).value( );
    // m_mesh               = load_mesh_from_file( "../../assets/cube/cube.gltf", "Cube.001" ).value( );
    m_mesh               = load_mesh_from_file( "../../assets/lucy/lucy.gltf", "Lucy_3M_O10" ).value( );
    m_mesh.vertex_buffer = create_buffer( m_mesh.vertices.size( ) * sizeof( Vertex ), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true, false );
    m_mesh.index_buffer  = create_buffer( m_mesh.indices.size( ) * sizeof( u32 ), VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY );

    build_meshlets( m_mesh );
    m_mesh.meshlets_buffer = create_buffer( m_mesh.meshlets.size( ) * sizeof( Meshlet ), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true, false );

    // upload mesh
    upload_buffer_data( g_ctx.staging_buffer, m_mesh.vertices.data( ), m_mesh.vertex_buffer.size );
    upload_buffer_data( g_ctx.staging_buffer, m_mesh.indices.data( ), m_mesh.index_buffer.size, m_mesh.vertex_buffer.size ); // Upload to the same staging buffer after the vertex data;
    upload_buffer_data( g_ctx.staging_buffer, m_mesh.meshlets.data( ), m_mesh.meshlets_buffer.size, m_mesh.vertex_buffer.size + m_mesh.index_buffer.size );

    {
        auto& cmd = g_ctx.global_cmd;
        VKCALL( vkResetFences( g_ctx.device, 1, &g_ctx.global_fence ) );
        VKCALL( vkResetCommandBuffer( cmd, 0 ) );
        begin_command( cmd, VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT );

        // Copy vertex and index buffer
        const VkBufferCopy vertex_copy = {
                .srcOffset = 0,
                .dstOffset = 0,
                .size      = m_mesh.vertex_buffer.size };
        const VkBufferCopy index_copy = {
                .srcOffset = m_mesh.vertex_buffer.size,
                .dstOffset = 0,
                .size      = m_mesh.index_buffer.size };
        vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, m_mesh.vertex_buffer.buffer, 1, &vertex_copy );
        vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, m_mesh.index_buffer.buffer, 1, &index_copy );

        // Copy meshlets
        const VkBufferCopy meshlets_copy = {
                .srcOffset = m_mesh.vertex_buffer.size + m_mesh.index_buffer.size,
                .dstOffset = 0,
                .size      = m_mesh.meshlets_buffer.size };
        vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, m_mesh.meshlets_buffer.buffer, 1, &meshlets_copy );

        VKCALL( vkEndCommandBuffer( cmd ) );
        submit_command( cmd, g_ctx.graphics_queue, g_ctx.global_fence );
        VKCALL( vkWaitForFences( g_ctx.device, 1, &g_ctx.global_fence, true, UINT64_MAX ) );
    }
}

void Renderer::Shutdown( ) {
    vkDeviceWaitIdle( g_ctx.device );

    destroy_mesh( m_mesh );

    m_pipeline.shutdown( );
    m_mesh_pipeline.shutdown( );
    g_ctx.shutdown( );
    SDL_DestroyWindow( m_window );
}

void Renderer::Run( ) {
    f64 avg_cpu_time = 0;
    f64 avg_gpu_time = 0;

    while ( !m_quit ) {
        m_frame_triangles   = 0;
        auto cpu_time_start = get_time( );

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
        vkResetQueryPool( g_ctx.device, frame.query_pool_timestamps, 0, frame.gpu_timestamps.size( ) );

        // Start command
        VKCALL( vkResetCommandBuffer( cmd, 0 ) );
        begin_command( cmd, VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT );
        vkCmdWriteTimestamp( cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, frame.query_pool_timestamps, 0 );

        image_barrier( cmd, g_ctx.swapchain.images[swapchain_image_idx],
                       VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED,
                       VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT_KHR, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL );
        tick( swapchain_image_idx );
        image_barrier( cmd, g_ctx.swapchain.images[swapchain_image_idx],
                       VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT_KHR, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                       VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT_KHR, VK_ACCESS_2_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR );

        vkCmdWriteTimestamp( cmd, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, frame.query_pool_timestamps, 1 );
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

        // Timers
        vkGetQueryPoolResults( g_ctx.device, frame.query_pool_timestamps, 0, u32( frame.gpu_timestamps.size( ) ), frame.gpu_timestamps.size( ) * sizeof( u64 ), frame.gpu_timestamps.data( ), sizeof( u64 ), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT );

        g_ctx.current_frame++;
        auto cpu_time_end = get_time( );

        avg_cpu_time = avg_cpu_time * 0.95 + ( ( cpu_time_end - cpu_time_start ) * 1000.0f ) * 0.05;
        avg_gpu_time = avg_gpu_time * 0.95 + ( g_ctx.get_query_time_in_ms( frame.gpu_timestamps[0], frame.gpu_timestamps[1] ) ) * 0.05f;

        f64 triangles_per_sec = f64( m_frame_triangles ) / f64( avg_cpu_time * 1e-3 );

        auto title = std::format( "cpu: {:.3f}; gpu: {:.3f}; triangles {:.2f}M; {:.1f}B tri/sec; pipeline: {}", avg_cpu_time, avg_gpu_time, f64( m_frame_triangles ) * 1e-6, triangles_per_sec * 1e-9, ( m_use_mesh_pipeline ? "meshlet" : "buffer" ) );
        SDL_SetWindowTitle( m_window, title.c_str( ) );
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

        if ( event.type == SDL_KEYDOWN ) {
            if ( event.key.keysym.scancode == SDL_SCANCODE_SPACE ) {
                m_use_mesh_pipeline = !m_use_mesh_pipeline;
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

    Pipeline& pipeline = m_use_mesh_pipeline ? m_mesh_pipeline : m_pipeline;
    vkCmdBindPipeline( cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline.pipeline );

    // Camera matrices
    glm::mat4 view       = glm::mat4_cast( m_camera.orientation );
    view[3]              = glm::vec4( m_camera.position, 1.0f );
    view                 = inverse( view );
    glm::mat4 projection = glm::perspective( m_camera.fov, ( float )width / ( float )height, m_camera.near, m_camera.far );

    ScenePushConstants pc{
            .view              = view,
            .projection        = projection,
            .camera_position   = glm::vec4( m_camera.position, 1.0f ),
            .vertex_buffer     = m_mesh.vertex_buffer.device_address,
            .meshlets_buffer   = m_mesh.meshlets_buffer.device_address,
            .meshlet_vertices  = m_mesh.meshlets_vertices.device_address,
            .meshlet_triangles = m_mesh.meshlets_triangles.device_address,
            .meshlet_count     = u32( m_mesh.meshlets.size( ) ) };

    if ( m_use_mesh_pipeline ) {
        vkCmdPushConstants( cmd, pipeline.layout, VK_SHADER_STAGE_ALL_GRAPHICS | VK_SHADER_STAGE_MESH_BIT_EXT | VK_SHADER_STAGE_TASK_BIT_EXT, 0, sizeof( ScenePushConstants ), &pc );
        for ( auto i = 0; i < 100; i++ ) {
            for ( auto& meshlet : m_mesh.meshlets ) {
                m_frame_triangles += meshlet.triangle_count;
            }

            auto n = m_mesh.meshlets.size( ) / 32;
            vkCmdDrawMeshTasksEXT( cmd, n, 1, 1 );
        }
    }
    else {
        vkCmdPushConstants( cmd, pipeline.layout, VK_SHADER_STAGE_ALL_GRAPHICS, 0, sizeof( ScenePushConstants ), &pc );
        vkCmdBindIndexBuffer( cmd, m_mesh.index_buffer.buffer, 0, VK_INDEX_TYPE_UINT32 );
        for ( auto i = 0; i < 100; i++ ) {
            vkCmdDrawIndexed( cmd, ( u32 )m_mesh.indices.size( ), 1, 0, 0, 0 );
            m_frame_triangles += m_mesh.indices.size( ) / 3;
        }
    }

    vkCmdEndRendering( cmd );
}

void tl::build_meshlets( Mesh& mesh ) {
    size_t                       max_meshlets = meshopt_buildMeshletsBound( mesh.indices.size( ), max_vertices, max_triangles );
    std::vector<meshopt_Meshlet> meshlets( max_meshlets );
    std::vector<unsigned int>    meshlet_vertices( max_meshlets * max_vertices );
    std::vector<unsigned char>   meshlet_triangles( max_meshlets * max_triangles * 3 );

    size_t meshlet_count = meshopt_buildMeshlets(
            meshlets.data( ),
            meshlet_vertices.data( ),
            meshlet_triangles.data( ),
            mesh.indices.data( ),
            mesh.indices.size( ),
            &mesh.vertices[0].vx,
            mesh.vertices.size( ),
            sizeof( Vertex ),
            max_vertices,
            max_triangles, 0.5f );

    std::println( "Meshlet count {}", meshlet_count );

    mesh.meshlets_vertices = create_buffer( meshlet_vertices.size( ) * sizeof( u32 ), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true );
    upload_buffer_data( g_ctx.staging_buffer, meshlet_vertices.data( ), meshlet_vertices.size( ) * sizeof( u32 ) );

    mesh.meshlets_triangles = create_buffer( meshlet_triangles.size( ) * sizeof( u8 ), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true );
    upload_buffer_data( g_ctx.staging_buffer, meshlet_triangles.data( ), meshlet_triangles.size( ) * sizeof( u8 ), meshlet_vertices.size( ) * sizeof( u32 ) );

    mesh.meshlets.clear( );
    mesh.meshlets.reserve( meshlet_count );

    for ( u32 i = 0; i < meshlet_count; i++ ) {
        auto& mopt_meshlet = meshlets[i];
        meshopt_optimizeMeshlet( &meshlet_vertices[mopt_meshlet.vertex_offset], &meshlet_triangles[mopt_meshlet.triangle_offset], mopt_meshlet.triangle_count, mopt_meshlet.vertex_count );
        auto bounds = meshopt_computeMeshletBounds( &meshlet_vertices[mopt_meshlet.vertex_offset], &meshlet_triangles[mopt_meshlet.triangle_offset], mopt_meshlet.triangle_count, &mesh.vertices[0].vx, mesh.vertices.size( ), sizeof( Vertex ) );

        Meshlet meshlet{ };
        meshlet.triangle_count  = mopt_meshlet.triangle_count;
        meshlet.vertex_count    = mopt_meshlet.vertex_count;
        meshlet.triangle_offset = mopt_meshlet.triangle_offset;
        meshlet.vertex_offset   = mopt_meshlet.vertex_offset;

        meshlet.cone_apex[0] = bounds.cone_apex[0];
        meshlet.cone_apex[1] = bounds.cone_apex[1];
        meshlet.cone_apex[2] = bounds.cone_apex[2];

        meshlet.cone_axis[0] = bounds.cone_axis[0];
        meshlet.cone_axis[1] = bounds.cone_axis[1];
        meshlet.cone_axis[2] = bounds.cone_axis[2];

        meshlet.cone_cutoff = bounds.cone_cutoff;

        mesh.meshlets.push_back( meshlet );
    }

    {
        auto& cmd = g_ctx.global_cmd;
        VKCALL( vkResetFences( g_ctx.device, 1, &g_ctx.global_fence ) );
        VKCALL( vkResetCommandBuffer( cmd, 0 ) );
        begin_command( cmd, VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT );

        // Copy vertex and index buffer
        const VkBufferCopy vertices_copy = {
                .srcOffset = 0,
                .dstOffset = 0,
                .size      = mesh.meshlets_vertices.size };
        const VkBufferCopy triangles_copy = {
                .srcOffset = mesh.meshlets_vertices.size,
                .dstOffset = 0,
                .size      = mesh.meshlets_triangles.size };
        vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, mesh.meshlets_vertices.buffer, 1, &vertices_copy );
        vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, mesh.meshlets_triangles.buffer, 1, &triangles_copy );

        VKCALL( vkEndCommandBuffer( cmd ) );
        submit_command( cmd, g_ctx.graphics_queue, g_ctx.global_fence );
        VKCALL( vkWaitForFences( g_ctx.device, 1, &g_ctx.global_fence, true, UINT64_MAX ) );
    }
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

void tl::destroy_mesh( Mesh& mesh ) {
    destroy_buffer( mesh.vertex_buffer );
    destroy_buffer( mesh.index_buffer );
    destroy_buffer( mesh.meshlets_buffer );
    destroy_buffer( mesh.meshlets_vertices );
    destroy_buffer( mesh.meshlets_triangles );
}