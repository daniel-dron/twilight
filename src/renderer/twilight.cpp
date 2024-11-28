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

    m_camera = Camera( glm::vec3( 0.0f, 0.0f, 0.0f ), 0, 0, width, height );

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

    m_drawcmd_pipeline.initialize( PipelineConfig{
            .name                 = "draw commands",
            .compute              = "../shaders/drawcmd.comp.spv",
            .push_constant_ranges = { VkPushConstantRange{ .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT, .size = sizeof( DrawCommandComputePushConstants ) } } } );

    m_command_buffer       = create_buffer( sizeof( DrawMeshTaskCommand ) * m_draws_count, VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true );
    m_command_count_buffer = create_buffer( sizeof( u32 ), VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true );
    m_draws_buffer         = create_buffer( sizeof( Draw ) * m_draws_count, VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true );

    // m_mesh_assets.emplace_back( load_mesh_from_file( "../../assets/lucy/lucy.gltf", "Lucy_3M_O10" ).value( ) );
    load_mesh_from_file( "../../assets/teapot/teapot.gltf", "Teapot", m_scene_geometry );
    load_mesh_from_file( "../../assets/cube/cube.gltf", "Cube.001", m_scene_geometry );

    // Upload scene geometry to the gpu
    _upload_scene_geometry( );

    u64 count  = m_draws_count;
    f32 radius = 500.0f;
    m_draws.reserve( count );

    std::random_device                    rd;
    std::mt19937                          gen( rd( ) );
    std::uniform_real_distribution<float> posDist( -radius, radius );
    std::uniform_real_distribution<float> rotDist( 0.0f, 360.0f );
    std::uniform_real_distribution<float> scaleDist( 0.8f, 1.2f );

    for ( int i = 0; i < count; ++i ) {
        u64 mesh_id = rand( ) % m_scene_geometry.meshes.size( );

        glm::vec3 position(
                posDist( gen ),
                posDist( gen ),
                posDist( gen ) );

        float rotX = glm::radians( rotDist( gen ) );
        float rotY = glm::radians( rotDist( gen ) );
        float rotZ = glm::radians( rotDist( gen ) );

        float scale = scaleDist( gen );
        if ( mesh_id == 0 ) {
            // scale /= 100.0f;
        }

        glm::mat4 model = glm::mat4( 1.0f );
        model           = glm::translate( model, position );
        model           = glm::rotate( model, rotX, glm::vec3( 1.0f, 0.0f, 0.0f ) );
        model           = glm::rotate( model, rotY, glm::vec3( 0.0f, 1.0f, 0.0f ) );
        model           = glm::rotate( model, rotZ, glm::vec3( 0.0f, 0.0f, 1.0f ) );
        model           = glm::scale( model, glm::vec3( scale ) );

        m_draws.emplace_back( Draw{ model, mesh_id } );
    }

    {
        auto& cmd = g_ctx.global_cmd;
        VKCALL( vkResetFences( g_ctx.device, 1, &g_ctx.global_fence ) );
        VKCALL( vkResetCommandBuffer( cmd, 0 ) );
        begin_command( cmd, VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT );

        upload_buffer_data( g_ctx.staging_buffer, m_draws.data( ), sizeof( Draw ) * m_draws.size( ) );

        const VkBufferCopy draws_copy = {
                .srcOffset = 0,
                .dstOffset = 0,
                .size      = sizeof( Draw ) * m_draws.size( ) };
        vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, m_draws_buffer.buffer, 1, &draws_copy );

        VKCALL( vkEndCommandBuffer( cmd ) );
        submit_command( cmd, g_ctx.graphics_queue, g_ctx.global_fence );
        VKCALL( vkWaitForFences( g_ctx.device, 1, &g_ctx.global_fence, true, UINT64_MAX ) );
    }
}

void Renderer::Shutdown( ) {
    vkDeviceWaitIdle( g_ctx.device );

    destroy_buffer( m_command_buffer );
    destroy_buffer( m_draws_buffer );
    destroy_buffer( m_command_count_buffer );
    destroy_buffer( m_scene_geometry.vertices_buffer );
    // destroy_buffer( m_scene_geometry.indices_buffer );
    destroy_buffer( m_scene_geometry.meshlets_buffer );
    destroy_buffer( m_scene_geometry.meshlet_data_buffer );
    destroy_buffer( m_scene_geometry.meshes_buffer );


    m_drawcmd_pipeline.shutdown( );
    m_mesh_pipeline.shutdown( );
    g_ctx.shutdown( );
    SDL_DestroyWindow( m_window );
}

void Renderer::Run( ) {
    f64 avg_cpu_time  = 0;
    f64 avg_gpu_time  = 0;
    f64 avg_cull_time = 0;

    while ( !m_quit ) {
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

        // Command Draws
        {
            vkCmdWriteTimestamp( cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, frame.query_pool_timestamps, 2 );

            // Reset command count buffer to 0 (using a staging buffer)
            u32 count = 0;
            upload_buffer_data( g_ctx.staging_buffer, &count, sizeof( u32 ) );

            const VkBufferCopy copy = {
                    .srcOffset = 0,
                    .dstOffset = 0,
                    .size      = sizeof( u32 ) };
            vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, m_command_count_buffer.buffer, 1, &copy );
            buffer_barrier( cmd, m_command_count_buffer.buffer, m_command_count_buffer.size, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_SHADER_READ_BIT );

            vkCmdBindPipeline( cmd, VK_PIPELINE_BIND_POINT_COMPUTE, m_drawcmd_pipeline.pipeline );
            DrawCommandComputePushConstants pc{
                    .draws             = m_draws_buffer.device_address,
                    .cmds              = m_command_buffer.device_address,
                    .meshes            = m_scene_geometry.meshes_buffer.device_address,
                    .count             = m_draws.size( ),
                    .draw_count_buffer = m_command_count_buffer.device_address,
            };

            if ( !m_freeze_frustum ) {
                m_current_frustum = m_camera.get_frustum( );
            }

            if ( m_culling ) {
                pc.frustum[0] = m_current_frustum.planes[0];
                pc.frustum[1] = m_current_frustum.planes[1];
                pc.frustum[2] = m_current_frustum.planes[2];
                pc.frustum[3] = m_current_frustum.planes[3];
                pc.frustum[4] = m_current_frustum.planes[4];
                pc.frustum[5] = m_current_frustum.planes[5];
            }

            vkCmdPushConstants( cmd, m_drawcmd_pipeline.layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( DrawCommandComputePushConstants ), &pc );
            vkCmdDispatch( cmd, u32( m_draws.size( ) + 31 ) / 32, 1, 1 );
            buffer_barrier( cmd, m_command_buffer.buffer, m_command_buffer.size, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT_KHR, VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT, VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT );

            vkCmdWriteTimestamp( cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, frame.query_pool_timestamps, 3 );
        }

        image_barrier( cmd, g_ctx.swapchain.images[swapchain_image_idx],
                       VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED,
                       VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT_KHR, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL );
        image_barrier( cmd, frame.depth.image, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT_KHR, 0, VK_IMAGE_LAYOUT_UNDEFINED,
                       VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT | VK_ACCESS_2_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                       VK_IMAGE_ASPECT_DEPTH_BIT );

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
        vkGetQueryPoolResults( g_ctx.device, frame.query_pool_timestamps, 0, 4, frame.gpu_timestamps.size( ) * sizeof( u64 ), frame.gpu_timestamps.data( ), sizeof( u64 ), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT );

        g_ctx.current_frame++;
        auto cpu_time_end = get_time( );

        avg_cpu_time  = avg_cpu_time * 0.95 + ( ( cpu_time_end - cpu_time_start ) * 1000.0f ) * 0.05;
        avg_gpu_time  = avg_gpu_time * 0.95 + ( g_ctx.get_query_time_in_ms( frame.gpu_timestamps[0], frame.gpu_timestamps[1] ) ) * 0.05f;
        avg_cull_time = avg_cull_time * 0.95 + ( g_ctx.get_query_time_in_ms( frame.gpu_timestamps[2], frame.gpu_timestamps[3] ) ) * 0.05f;

        f64 triangles_per_sec = f64( m_frame_triangles ) / f64( avg_cpu_time * 1e-3 );

        auto title = std::format( "cpu: {:.3f}; gpu: {:.3f}; cull: {:.3f}; triangles {:.2f}M; {:.1f}B tri/sec; draws: {}; culling: {}{};",
                                  avg_cpu_time, avg_gpu_time, avg_cull_time, f64( m_frame_triangles ) * 1e-6, triangles_per_sec * 1e-9, m_draws_count, ( m_culling ) ? "ON" : "OFF", ( m_freeze_frustum ) ? " (FROZEN)" : "" );
        SDL_SetWindowTitle( m_window, title.c_str( ) );
    }
}

void Renderer::tick( u32 swapchain_image_idx ) {
    auto& frame = g_ctx.get_current_frame( );
    auto& cmd   = frame.cmd;

    VkClearValue              clear_color{ 0.05f, 0.1f, 0.3f, 1.0f };
    std::array                attachments = { attachment( g_ctx.swapchain.views[swapchain_image_idx], &clear_color ) };
    VkRenderingAttachmentInfo depth_attachment{
            .sType       = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
            .pNext       = nullptr,
            .imageView   = frame.depth.view,
            .imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
            .loadOp      = VK_ATTACHMENT_LOAD_OP_CLEAR,
            .storeOp     = VK_ATTACHMENT_STORE_OP_STORE,
            .clearValue  = {
                     .depthStencil = { .depth = 1.0f } } };
    VkRenderingInfo render_info = {
            .sType                = VK_STRUCTURE_TYPE_RENDERING_INFO,
            .pNext                = nullptr,
            .renderArea           = VkRect2D{ VkOffset2D{ 0, 0 }, VkExtent2D{ width, height } },
            .layerCount           = 1,
            .colorAttachmentCount = ( u32 )attachments.size( ),
            .pColorAttachments    = attachments.data( ),
            .pDepthAttachment     = &depth_attachment };
    vkCmdBeginRendering( cmd, &render_info );

    VkViewport viewport = {
            .x = 0, .y = 0, .width = ( float )width, .height = ( float )height, .minDepth = 0.0f, .maxDepth = 1.0f };
    vkCmdSetViewport( cmd, 0, 1, &viewport );

    const VkRect2D scissor = {
            .offset = { .x = 0, .y = 0 },
            .extent = { .width = width, .height = height } };
    vkCmdSetScissor( cmd, 0, 1, &scissor );

    vkCmdBindPipeline( cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_mesh_pipeline.pipeline );

    // Camera matrices
    ScenePushConstants pc{
            .view                 = m_camera.get_view_matrix( ),
            .projection           = m_camera.get_projection_matrix( ),
            .camera_position      = glm::vec4( m_camera.get_position( ), 1.0f ),
            .draws_buffer         = m_draws_buffer.device_address,
            .meshes               = m_scene_geometry.meshes_buffer.device_address,
            .meshlets_buffer      = m_scene_geometry.meshlets_buffer.device_address,
            .meshlets_data_buffer = m_scene_geometry.meshlet_data_buffer.device_address,
            .vertex_buffer        = m_scene_geometry.vertices_buffer.device_address,
            .draw_cmds            = m_command_buffer.device_address,
    };

    vkCmdPushConstants( cmd, m_mesh_pipeline.layout, VK_SHADER_STAGE_ALL_GRAPHICS | VK_SHADER_STAGE_MESH_BIT_EXT | VK_SHADER_STAGE_TASK_BIT_EXT, 0, sizeof( ScenePushConstants ), &pc );
    // vkCmdDrawMeshTasksIndirectEXT( cmd, m_command_buffer.buffer, 0, m_draws_count, sizeof( DrawMeshTaskCommand ) );
    vkCmdDrawMeshTasksIndirectCountEXT( cmd, m_command_buffer.buffer, 0, m_command_count_buffer.buffer, 0, m_draws_count, sizeof( DrawMeshTaskCommand ) );

    vkCmdEndRendering( cmd );
}

void Renderer::_upload_scene_geometry( ) {
    // Create buffers
    u64 vertex_size                  = m_scene_geometry.vertices.size( ) * sizeof( Vertex );
    m_scene_geometry.vertices_buffer = create_buffer( vertex_size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true );

    // u64 indices_size = m_scene_geometry.indices.size( ) * sizeof( u32 );
    // m_scene_geometry.indices_buffer = create_buffer(u64 size, VkBufferUsageFlags usage, VmaAllocationCreateFlags vma_flags, VmaMemoryUsage vma_usage)

    u64 meshlets_size                = m_scene_geometry.meshlets.size( ) * sizeof( Meshlet );
    m_scene_geometry.meshlets_buffer = create_buffer( meshlets_size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true );

    u64 meshlet_data_size                = m_scene_geometry.meshlet_data.size( ) * sizeof( u32 );
    m_scene_geometry.meshlet_data_buffer = create_buffer( meshlet_data_size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true );

    u64 meshes_size                = m_scene_geometry.meshes.size( ) * sizeof( Mesh );
    m_scene_geometry.meshes_buffer = create_buffer( meshes_size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true );

    // upload to staging buffer first
    upload_buffer_data( g_ctx.staging_buffer, m_scene_geometry.vertices.data( ), vertex_size );
    upload_buffer_data( g_ctx.staging_buffer, m_scene_geometry.meshlets.data( ), meshlets_size, vertex_size );
    upload_buffer_data( g_ctx.staging_buffer, m_scene_geometry.meshlet_data.data( ), meshlet_data_size, vertex_size + meshlets_size );
    upload_buffer_data( g_ctx.staging_buffer, m_scene_geometry.meshes.data( ), meshes_size, vertex_size + meshlets_size + meshlet_data_size );

    // transfer from staging to each buffer
    {
        auto& cmd = g_ctx.global_cmd;
        VKCALL( vkResetFences( g_ctx.device, 1, &g_ctx.global_fence ) );
        VKCALL( vkResetCommandBuffer( cmd, 0 ) );
        begin_command( cmd, VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT );

        const VkBufferCopy vertices_copy = {
                .srcOffset = 0,
                .dstOffset = 0,
                .size      = vertex_size,
        };
        const VkBufferCopy meshlets_copy = {
                .srcOffset = vertex_size,
                .dstOffset = 0,
                .size      = meshlets_size,
        };
        const VkBufferCopy meshlet_data_copy = {
                .srcOffset = vertex_size + meshlets_size,
                .dstOffset = 0,
                .size      = meshlet_data_size,
        };
        const VkBufferCopy meshes_copy = {
                .srcOffset = vertex_size + meshlets_size + meshlet_data_size,
                .dstOffset = 0,
                .size      = meshes_size,
        };

        vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, m_scene_geometry.vertices_buffer.buffer, 1, &vertices_copy );
        vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, m_scene_geometry.meshlets_buffer.buffer, 1, &meshlets_copy );
        vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, m_scene_geometry.meshlet_data_buffer.buffer, 1, &meshlet_data_copy );
        vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, m_scene_geometry.meshes_buffer.buffer, 1, &meshes_copy );

        buffer_barrier( cmd, m_scene_geometry.vertices_buffer.buffer, m_scene_geometry.vertices_buffer.size, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT );
        buffer_barrier( cmd, m_scene_geometry.meshlets_buffer.buffer, m_scene_geometry.meshlets_buffer.size, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT );
        buffer_barrier( cmd, m_scene_geometry.meshlet_data_buffer.buffer, m_scene_geometry.meshlet_data_buffer.size, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT );
        buffer_barrier( cmd, m_scene_geometry.meshes_buffer.buffer, m_scene_geometry.meshes_buffer.size, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT );

        VKCALL( vkEndCommandBuffer( cmd ) );
        submit_command( cmd, g_ctx.graphics_queue, g_ctx.global_fence );
        VKCALL( vkWaitForFences( g_ctx.device, 1, &g_ctx.global_fence, true, UINT64_MAX ) );
    }
}

void Renderer::process_events( ) {
    SDL_Event    event;
    const Uint8* keystate = SDL_GetKeyboardState( NULL );

    while ( SDL_PollEvent( &event ) != 0 ) {
        if ( event.type == SDL_QUIT ) {
            m_quit = true;
        }

        if ( event.type == SDL_WINDOWEVENT ) {
            if ( event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED && event.window.data1 > 0 && event.window.data2 > 0 ) {
                this->width  = event.window.data1;
                this->height = event.window.data2;
                g_ctx.resize( event.window.data1, event.window.data2, g_ctx.device, g_ctx.surface );
            }
        }

        if ( event.type == SDL_KEYDOWN ) {
            if ( event.key.keysym.scancode == SDL_SCANCODE_C ) {
                m_culling = !m_culling;
            }
            else if ( event.key.keysym.scancode == SDL_SCANCODE_F ) {
                m_freeze_frustum = !m_freeze_frustum;
            }
        }

        if ( event.type == SDL_MOUSEMOTION ) {
            if ( SDL_GetMouseState( NULL, NULL ) & SDL_BUTTON( SDL_BUTTON_RIGHT ) ) {
                float sensitivity = 0.2f;
                float delta_yaw   = event.motion.xrel * sensitivity;
                float delta_pitch = -event.motion.yrel * sensitivity;
                m_camera.rotate( delta_yaw, delta_pitch );
            }
        }
    }

    glm::vec3 movement( 0.0f );
    if ( keystate[SDL_SCANCODE_W] )
        movement += m_camera.get_front( );
    if ( keystate[SDL_SCANCODE_S] )
        movement -= m_camera.get_front( );
    if ( keystate[SDL_SCANCODE_A] )
        movement -= m_camera.get_right( );
    if ( keystate[SDL_SCANCODE_D] )
        movement += m_camera.get_right( );
    if ( glm::length( movement ) > 0 ) {
        glm::vec3 current_pos = m_camera.get_position( );
        m_camera.set_position( current_pos + glm::normalize( movement ) * move_speed );
    }
}

void tl::build_meshlets( const std::vector<Vertex>& vertices, const std::vector<u32>& indices, SceneGeometry& scene ) {
    size_t                       max_meshlets = meshopt_buildMeshletsBound( indices.size( ), max_vertices, max_triangles );
    std::vector<meshopt_Meshlet> meshlets( max_meshlets );
    std::vector<unsigned int>    meshlet_vertices( max_meshlets * max_vertices );
    std::vector<unsigned char>   meshlet_triangles( max_meshlets * max_triangles * 3 );

    size_t meshlet_count = meshopt_buildMeshlets(
            meshlets.data( ),
            meshlet_vertices.data( ),
            meshlet_triangles.data( ),
            indices.data( ),
            indices.size( ),
            &vertices[0].vx,
            vertices.size( ),
            sizeof( Vertex ),
            max_vertices,
            max_triangles, 0.5f );

    std::println( "Meshlet count {}", meshlet_count );

    for ( auto& mopt_meshlet : meshlets ) {
        meshopt_optimizeMeshlet( &meshlet_vertices[mopt_meshlet.vertex_offset], &meshlet_triangles[mopt_meshlet.triangle_offset], mopt_meshlet.triangle_count, mopt_meshlet.vertex_count );

        u64 data_offset = scene.meshlet_data.size( );

        for ( u32 i = 0; i < mopt_meshlet.vertex_count; i++ ) {
            scene.meshlet_data.push_back( meshlet_vertices[i + mopt_meshlet.vertex_offset] );
        }

        for ( u32 i = 0; i < mopt_meshlet.triangle_count * 3; i++ ) {
            scene.meshlet_data.push_back( meshlet_triangles[i + mopt_meshlet.triangle_offset] );
        }

        auto bounds = meshopt_computeMeshletBounds( &meshlet_vertices[mopt_meshlet.vertex_offset], &meshlet_triangles[mopt_meshlet.triangle_offset], mopt_meshlet.triangle_count, &vertices[0].vx, vertices.size( ), sizeof( Vertex ) );

        Meshlet meshlet{ };
        meshlet.data_offset    = data_offset;
        meshlet.vertex_count   = mopt_meshlet.vertex_count;
        meshlet.triangle_count = mopt_meshlet.triangle_count;

        meshlet.cone_apex[0] = bounds.cone_apex[0];
        meshlet.cone_apex[1] = bounds.cone_apex[1];
        meshlet.cone_apex[2] = bounds.cone_apex[2];

        meshlet.cone_axis[0] = bounds.cone_axis[0];
        meshlet.cone_axis[1] = bounds.cone_axis[1];
        meshlet.cone_axis[2] = bounds.cone_axis[2];

        meshlet.cone_cutoff = bounds.cone_cutoff;

        scene.meshlets.emplace_back( meshlet );
    }
}

u32 tl::load_mesh_from_file( const std::string& gltf_path, const std::string& mesh_name, SceneGeometry& scene ) {
    assert( !gltf_path.empty( ) );
    assert( !mesh_name.empty( ) );
    assert( std::filesystem::exists( gltf_path ) );

    Assimp::Importer importer;
    const auto       aiScene = importer.ReadFile( gltf_path, aiProcess_Triangulate | aiProcess_GenBoundingBoxes | aiProcess_FlipWindingOrder );

    for ( size_t i = 0; i < aiScene->mNumMeshes; i++ ) {
        if ( std::string( aiScene->mMeshes[i]->mName.C_Str( ) ) == mesh_name ) {
            auto ai_mesh = aiScene->mMeshes[i];

            std::vector<Vertex> vertices;
            vertices.reserve( ai_mesh->mNumVertices );
            for ( u32 i = 0; i < ai_mesh->mNumVertices; i++ ) {
                auto vertex  = ai_mesh->mVertices[i];
                auto normals = ai_mesh->mNormals[i];

                vertices.push_back( { vertex.x, vertex.y, vertex.z, normals.x, normals.y, normals.z } );
            }

            std::vector<u32> indices;
            indices.reserve( ai_mesh->mNumFaces * 3 );
            for ( u32 i = 0; i < ai_mesh->mNumFaces; i++ ) {
                auto& face = ai_mesh->mFaces[i];
                indices.emplace_back( face.mIndices[0] );
                indices.emplace_back( face.mIndices[1] );
                indices.emplace_back( face.mIndices[2] );
            }

            // insert new indices and vertices into the global scene
            u32 vertex_offset = scene.vertices.size( );
            scene.vertices.insert( scene.vertices.end( ), vertices.begin( ), vertices.end( ) );
            scene.indices.insert( scene.indices.end( ), indices.begin( ), indices.end( ) );

            // generate meshlets and get offset into global scene
            u32 meshlet_index = scene.meshlets.size( );
            build_meshlets( vertices, indices, scene );
            u32 meshlet_count = scene.meshlets.size( ) - meshlet_index;

            // bounding sphere
            auto      min    = glm::vec3( ai_mesh->mAABB.mMin.x, ai_mesh->mAABB.mMin.y, ai_mesh->mAABB.mMin.z );
            auto      max    = glm::vec3( ai_mesh->mAABB.mMax.x, ai_mesh->mAABB.mMax.y, ai_mesh->mAABB.mMax.z );
            glm::vec3 center = ( min + max ) * 0.5f;
            f32       radius = glm::length( max - center );

            // Place mesh into scene
            Mesh mesh{
                    .center = center,
                    .radius = radius,

                    .vertex_offset = vertex_offset,
                    .meshlet_index = meshlet_index,
                    .meshlet_count = meshlet_count,
            };
            scene.meshes.emplace_back( mesh );

            return scene.meshes.size( ) - 1;
        }
    }

    return -1;
}