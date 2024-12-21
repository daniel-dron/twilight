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
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <meshoptimizer.h>
#include <pch.h>
#include "SDL.h"
#include "SDL_events.h"
#include "SDL_scancode.h"
#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"
#include "r_resources.h"
#include "r_shaders.h"
#include "renderer/r_resources.h"
#include "types.h"

#include <renderer/r_context.h>
#include <renderer/r_shaders.h>
#include <string>
#include <vector>
#include <vulkan/vulkan_core.h>

using namespace tl;

void Renderer::Initialize( int count ) {
    if ( count > 0 ) {
        m_draws_count = count;
    }

    // Initialize SDL
    SDL_Init( SDL_INIT_VIDEO );
    SDL_WindowFlags flags = ( SDL_WindowFlags )( SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE );

    m_window = SDL_CreateWindow( "Twilight", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, flags );

    g_ctx.initialize( width, height, "Twilight", m_window );

    m_camera = Camera( glm::vec3( 0.0f, 0.0f, 0.0f ), 0, 0, width, height );

    m_mesh_pipeline.initialize( PipelineConfig{
            .name                 = "mesh",
            .pixel                = "../shaders/meshlet.frag.slang.spv",
            .mesh                 = "../shaders/meshlet.mesh.slang.spv",
            .task                 = "../shaders/meshlet.task.slang.spv",
            .cull_mode            = VK_CULL_MODE_BACK_BIT,
            .front_face           = VK_FRONT_FACE_COUNTER_CLOCKWISE,
            .depth_compare        = VK_COMPARE_OP_GREATER_OR_EQUAL,
            .color_targets        = { PipelineConfig::ColorTargetsConfig{ .format = VK_FORMAT_R32G32B32A32_SFLOAT, .blend_type = PipelineConfig::BlendType::OFF } },
            .push_constant_ranges = { VkPushConstantRange{ .stageFlags = VK_SHADER_STAGE_ALL_GRAPHICS | VK_SHADER_STAGE_MESH_BIT_EXT | VK_SHADER_STAGE_TASK_BIT_EXT, .size = sizeof( ScenePushConstants ) } },
    } );

    m_early_cull_pipeline.initialize( PipelineConfig{
            .name                 = "early cull commands",
            .compute              = "../shaders/early_cull.comp.slang.spv",
            .push_constant_ranges = { VkPushConstantRange{ .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT, .size = sizeof( DrawCommandComputePushConstants ) } } } );

    m_late_cull_pipeline.initialize( PipelineConfig{
            .name                 = "late cull commands",
            .compute              = "../shaders/late_cull.comp.slang.spv",
            .push_constant_ranges = { VkPushConstantRange{ .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT, .size = sizeof( DrawCommandComputePushConstants ) } },
    } );
    m_late_cull_set = allocate_descript_set( g_ctx.descriptor_pool, m_late_cull_pipeline.bindings[BindingsStage::COMPUTE].layout, g_ctx.frame_overlap );

    m_depthpyramid_pipeline.initialize( PipelineConfig{
            .name                 = "depth pyramid",
            .compute              = "../shaders/depth_pyramid.comp.slang.spv",
            .push_constant_ranges = { VkPushConstantRange{ .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT, .size = sizeof( DepthPyramidPushConstants ) } },
    } );
    m_depthpyramid_sets = allocate_descript_set( g_ctx.descriptor_pool, m_depthpyramid_pipeline.bindings[BindingsStage::COMPUTE].layout, g_ctx.frame_overlap );

    m_linear_sampler    = create_sampler( );
    m_reduction_sampler = create_reduction_sampler( );

    m_command_buffer       = create_buffer( sizeof( DrawMeshTaskCommand ) * m_draws_count, VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true );
    m_command_count_buffer = create_buffer( sizeof( u32 ), VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true );
    m_draws_buffer         = create_buffer( sizeof( Draw ) * m_draws_count, VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true );
    m_visible_draws        = {
            create_buffer( sizeof( u32 ) * m_draws_count, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true ),
            create_buffer( sizeof( u32 ) * m_draws_count, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true ),
            create_buffer( sizeof( u32 ) * m_draws_count, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY ) };

    m_cull_data = {
            create_buffer( sizeof( CullData ), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, VMA_ALLOCATION_CREATE_MAPPED_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU, true, true ),
            create_buffer( sizeof( CullData ), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, VMA_ALLOCATION_CREATE_MAPPED_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU, true, true ) };

    // load_mesh_from_file( "../../assets/lucy/lucy.gltf", "Lucy_3M_O10", m_scene_geometry );
    // load_mesh_from_file( "../../assets/teapot/teapot.gltf", "Teapot", m_scene_geometry );
    // load_mesh_from_file( "../../assets/guanyin/scene.gltf", "Object_0", m_scene_geometry );
    load_mesh_from_file( "../../assets/cube/cube.gltf", "Cube.001", m_scene_geometry );

    // Upload scene geometry to the gpu
    _upload_scene_geometry( );

    // float     scale = 5.0f;
    // glm::mat4 model = glm::translate( glm::mat4( 1.0f ), glm::vec3( 15.0f, 0.0f, 0.0f ) );
    // model           = glm::scale( model, glm::vec3( scale ) );
    // m_draws.emplace_back( Draw{ model, 0 } );

    // model = glm::translate( glm::mat4( 1.0f ), glm::vec3( 25.0f, 0.0f, 0.0f ) );
    // m_draws.emplace_back( Draw{ model, 0 } );

    std::random_device rd;
    std::mt19937       gen( rd( ) );

    m_draws.reserve( m_draws_count );

    float base_mesh_size     = 5.0f;
    float avg_scale          = ( 0.1f + 2.0f ) / 2.0f;
    float avg_mesh_size      = base_mesh_size * avg_scale;
    float spacing_multiplier = 4.0f;
    float radius             = powf( powf( avg_mesh_size * spacing_multiplier, 3 ) * m_draws_count / 8.0f, 1.0f / 3.0f );

    std::uniform_real_distribution<float> pos_dist( -radius, radius );
    std::uniform_real_distribution<float> rot_dist( 0.0f, 360.0f );
    std::uniform_real_distribution<float> scale_dist( 0.1f, 2.0f );

    for ( int i = 0; i < m_draws_count; ++i ) {
        u64 mesh_id = rand( ) % m_scene_geometry.meshes.size( );

        glm::vec3 position(
                pos_dist( gen ),
                pos_dist( gen ),
                pos_dist( gen ) );

        float rotX = glm::radians( rot_dist( gen ) );
        float rotY = glm::radians( rot_dist( gen ) );
        float rotZ = glm::radians( rot_dist( gen ) );

        float scale = scale_dist( gen );
        // if ( mesh_id == 0 ) {
        // scale /= 100.0f;
        // }

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

    for ( auto& buff : m_visible_draws ) {
        destroy_buffer( buff );
    }

    destroy_buffer( m_command_count_buffer );
    destroy_buffer( m_scene_geometry.vertices_buffer );
    // destroy_buffer( m_scene_geometry.indices_buffer );
    destroy_buffer( m_scene_geometry.meshlets_buffer );
    destroy_buffer( m_scene_geometry.meshlet_data_buffer );
    destroy_buffer( m_scene_geometry.meshes_buffer );
    destroy_buffer( m_cull_data[0] );
    destroy_buffer( m_cull_data[1] );

    m_early_cull_pipeline.shutdown( );
    m_late_cull_pipeline.shutdown( );
    m_mesh_pipeline.shutdown( );
    m_depthpyramid_pipeline.shutdown( );

    for ( auto& set : m_depthpyramid_sets ) {
        vkFreeDescriptorSets( g_ctx.device, g_ctx.descriptor_pool, 1, &set );
    }

    vkDestroySampler( g_ctx.device, m_linear_sampler, nullptr );
    vkDestroySampler( g_ctx.device, m_reduction_sampler, nullptr );

    g_ctx.shutdown( );
    SDL_DestroyWindow( m_window );
}

void Renderer::Run( ) {
    f64 avg_cpu_time       = 0;
    f64 avg_gpu_time       = 0;
    f64 avg_cull_time      = 0;
    f64 avg_late_cull_time = 0;

    m_last_frame_time = get_time( );

    while ( !m_quit ) {
        f64 current_time  = get_time( );
        m_delta_time      = current_time - m_last_frame_time;
        m_last_frame_time = current_time;

        auto cpu_time_start = get_time( );

        process_events( );

        auto& frame               = g_ctx.get_current_frame( );
        auto  cmd                 = frame.cmd;
        auto& visible_draws       = m_visible_draws[g_ctx.get_current_frame_index( )];
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
        vkCmdWriteTimestamp( cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, frame.query_pool_timestamps, GPU_TOTAL_START );

        // For the FIRST frame, no meshes were visible last frame, so fill the buffer with 0s
        if ( g_ctx.current_frame == 0 ) {
            vkCmdFillBuffer( cmd, m_visible_draws[0].buffer, 0, m_visible_draws[0].size, 0 );
            vkCmdFillBuffer( cmd, m_visible_draws[1].buffer, 0, m_visible_draws[1].size, 0 );
            vkCmdFillBuffer( cmd, m_visible_draws[2].buffer, 0, m_visible_draws[2].size, 0 );

            buffer_barrier( cmd, m_visible_draws[0].buffer, m_visible_draws[0].size, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_READ_BIT );
            buffer_barrier( cmd, m_visible_draws[1].buffer, m_visible_draws[1].size, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_READ_BIT );
        }

        // STEP 0
        // On the beggining of the frame, copy from the staging visibility buffer (since we are using double inflight frames)
        buffer_barrier( cmd, visible_draws.buffer, visible_draws.size, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_MEMORY_READ_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT );
        buffer_barrier( cmd, m_visible_draws[2].buffer, visible_draws.size, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_MEMORY_READ_BIT );
        const VkBufferCopy copy_from = {
                .srcOffset = 0,
                .dstOffset = 0,
                .size      = m_visible_draws[2].size };
        vkCmdCopyBuffer( cmd, m_visible_draws[2].buffer, visible_draws.buffer, 1, &copy_from );

        // STEP 1 [Early Cull].
        // Fill draw calls for objects that WERE visible last frame (visibility = 1)
        // For the first frame, that should be no meshes.
        buffer_barrier( cmd, visible_draws.buffer, visible_draws.size, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_READ_BIT );
        buffer_barrier( cmd, m_command_buffer.buffer, m_command_buffer.size, VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT, VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT );
        _early_cull( );

        // STEP 2 [Early Draws]
        // Render the objects that passed the early cull step
        image_barrier( cmd, frame.depth.image, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT_KHR, 0, VK_IMAGE_LAYOUT_UNDEFINED, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT | VK_ACCESS_2_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, VK_IMAGE_ASPECT_DEPTH_BIT );
        image_barrier( cmd, frame.color.image, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL );
        buffer_barrier( cmd, m_command_buffer.buffer, m_command_buffer.size, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT_KHR, VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT, VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT );
        _issue_draw_calls( swapchain_image_idx );

        VkBufferCopy draws_copy = {
                .srcOffset = 0,
                .dstOffset = 0,
                .size      = sizeof( u32 ) };
        vkCmdCopyBuffer( cmd, m_command_count_buffer.buffer, g_ctx.readback_buffer.buffer, 1, &draws_copy );

        // STEP 3 [Depth Pyramid Construction]
        if ( !m_lock_occlusion ) {
            _construct_depth_pyramid( );
            m_occlusion_view        = m_camera.get_view_matrix( );
            m_occlusion_perspective = m_camera.get_projection_matrix( );
        }

        // STEP 4 [Late Cull]
        // Frustum cull + Occlusion Cull
        // Fill draw calls for objects that were NOT visible last frame (visibility = 0)
        buffer_barrier( cmd, visible_draws.buffer, visible_draws.size, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_READ_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT );
        buffer_barrier( cmd, m_command_buffer.buffer, m_command_buffer.size, VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT, VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT );
        _late_cull( );

        // STEP 5 [Late Draws]
        // Render the objects that passed the late cull step
        buffer_barrier( cmd, m_command_buffer.buffer, m_command_buffer.size, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT_KHR, VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT, VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT );
        _issue_draw_calls( swapchain_image_idx, false, false ); // Dont clear color and depth

        draws_copy = {
                .srcOffset = 0,
                .dstOffset = sizeof( u32 ),
                .size      = sizeof( u32 ) };
        vkCmdCopyBuffer( cmd, m_command_count_buffer.buffer, g_ctx.readback_buffer.buffer, 1, &draws_copy );

        // STEP 6
        // Copy visibility buffer from this frame to staging visibility buffer
        // On the beggining of the next frame, copy from it
        buffer_barrier( cmd, visible_draws.buffer, visible_draws.size, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT );
        buffer_barrier( cmd, m_visible_draws[2].buffer, visible_draws.size, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_MEMORY_READ_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT );
        const VkBufferCopy copy_to = {
                .srcOffset = 0,
                .dstOffset = 0,
                .size      = m_visible_draws[2].size };
        vkCmdCopyBuffer( cmd, visible_draws.buffer, m_visible_draws[2].buffer, 1, &copy_to );

        image_barrier( cmd, frame.depth_pyramid.image, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT | VK_ACCESS_2_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_ASPECT_COLOR_BIT, 0, VK_REMAINING_MIP_LEVELS );
        image_barrier( cmd, frame.color.image, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL );
        auto depth_levels = frame.depth_pyramid_levels;
        auto depth_width  = g_ctx.swapchain.width / depth_levels - 5;
        for ( u32 i = 0; i < depth_levels; i++ ) {
            VkImageBlit region = {
                    .srcSubresource = { .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .mipLevel = u32( i ), .layerCount = 1 },
                    .srcOffsets     = {
                            { 0, 0, 0 },
                            { i32( frame.depth_pyramid_size >> i ), i32( frame.depth_pyramid_size >> i ), 1 } },
                    .dstSubresource = { .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .layerCount = 1 },

                    .dstOffsets = {
                            { i32( i * ( depth_width + 5 ) ), 0, 0 },
                            { i32( depth_width + ( i * ( depth_width + 5 ) ) ), i32( depth_width ), 1 },
                    },
            };
            vkCmdBlitImage( cmd, frame.depth_pyramid.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, frame.color.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region, VK_FILTER_NEAREST );
        }

        // Copy from color target image to swapchain
        {
            // transition swapchain image for a copy as the destination
            image_barrier( cmd, g_ctx.swapchain.images[swapchain_image_idx], VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT_KHR, 0, VK_IMAGE_LAYOUT_UNDEFINED, VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL );
            // transition color target image for a copy as the source
            image_barrier( cmd, frame.color.image, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL );

            VkImageBlit region = {
                    .srcSubresource = { .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .layerCount = 1 },
                    .srcOffsets     = {
                            { 0, 0, 0 },
                            { i32( g_ctx.swapchain.width ), i32( g_ctx.swapchain.height ), 1 } },
                    .dstSubresource = { .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .layerCount = 1 },
                    .dstOffsets     = { { 0, 0, 0 }, { i32( g_ctx.swapchain.width ), i32( g_ctx.swapchain.height ), 1 } },
            };

            vkCmdBlitImage( cmd, frame.color.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, g_ctx.swapchain.images[swapchain_image_idx], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region, VK_FILTER_NEAREST );

            // transition the swapchain image to be presented
            image_barrier( cmd, g_ctx.swapchain.images[swapchain_image_idx], VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT_KHR, VK_ACCESS_2_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR );
        }

        vkCmdWriteTimestamp( cmd, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, frame.query_pool_timestamps, GPU_TOTAL_END );
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
        vkGetQueryPoolResults( g_ctx.device, frame.query_pool_timestamps, 0, 6, frame.gpu_timestamps.size( ) * sizeof( u64 ), frame.gpu_timestamps.data( ), sizeof( u64 ), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT );

        auto after_culling_count = read_from_buffer<u32>( g_ctx.readback_buffer ) + read_from_buffer<u32>( g_ctx.readback_buffer, sizeof( u32 ) );

        g_ctx.current_frame++;
        auto cpu_time_end = get_time( );

        avg_cpu_time       = avg_cpu_time * 0.95 + ( ( cpu_time_end - cpu_time_start ) * 1000.0f ) * 0.05;
        avg_gpu_time       = avg_gpu_time * 0.95 + ( g_ctx.get_query_time_in_ms( frame.gpu_timestamps[GPU_TOTAL_START], frame.gpu_timestamps[GPU_TOTAL_END] ) ) * 0.05f;
        avg_cull_time      = avg_cull_time * 0.95 + ( g_ctx.get_query_time_in_ms( frame.gpu_timestamps[GPU_TOTAL_FIRST_CULL_STEP_START], frame.gpu_timestamps[GPU_TOTAL_FIRST_CULL_STEP_END] ) ) * 0.05f;
        avg_late_cull_time = avg_late_cull_time * 0.95 + ( g_ctx.get_query_time_in_ms( frame.gpu_timestamps[GPU_TOTAL_SECOND_CULL_STEP_START], frame.gpu_timestamps[GPU_TOTAL_SECOND_CULL_STEP_END] ) ) * 0.05f;

        f64   triangles_per_sec = f64( m_frame_triangles ) / f64( avg_cpu_time * 1e-3 );
        auto& front             = m_camera.get_front( );
        auto  title             = std::format( "cpu: {:.3f}; gpu: {:.3f}; e_cull: {:.3f}; l_cull: {:.3f}; draws: {} {}; [C/F] frustum: {}{}; [O/P] occlusion: {}{};",
                                               avg_cpu_time, avg_gpu_time, avg_cull_time, avg_late_cull_time, m_draws_count, after_culling_count, ( m_culling ) ? "ON" : "OFF", ( m_freeze_frustum ) ? " (FROZEN)" : "", m_occlusion ? "ON" : "OFF", ( m_lock_occlusion ) ? " (FROZEN)" : "" );
        SDL_SetWindowTitle( m_window, title.c_str( ) );
    }
}

void Renderer::_issue_draw_calls( u32 swapchain_image_idx, bool b_clear_color, bool b_clear_depth ) {
    auto& frame = g_ctx.get_current_frame( );
    auto& cmd   = frame.cmd;

    VkClearValue              clear_color{ 0.05f, 0.1f, 0.3f, 1.0f };
    std::array                attachments = { attachment( frame.color.view, b_clear_color ? &clear_color : nullptr ) };
    VkRenderingAttachmentInfo depth_attachment{
            .sType       = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
            .pNext       = nullptr,
            .imageView   = frame.depth.view,
            .imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
            .loadOp      = b_clear_depth ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD,
            .storeOp     = VK_ATTACHMENT_STORE_OP_STORE,
            .clearValue  = {
                     .depthStencil = { .depth = 0.0f } } };
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
    vkCmdDrawMeshTasksIndirectCountEXT( cmd, m_command_buffer.buffer, 0, m_command_count_buffer.buffer, 0, m_draws_count, sizeof( DrawMeshTaskCommand ) );

    vkCmdEndRendering( cmd );
}

void Renderer::_early_cull( ) {
    auto& frame         = g_ctx.get_current_frame( );
    auto& cmd           = frame.cmd;
    auto& visible_draws = m_visible_draws[g_ctx.get_current_frame_index( )];

    vkCmdWriteTimestamp( cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, frame.query_pool_timestamps, GPU_TOTAL_FIRST_CULL_STEP_START );

    // Reset command count buffer to 0 (using a staging buffer)
    buffer_barrier( cmd, m_command_count_buffer.buffer, m_command_count_buffer.size, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_SHADER_READ_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT );
    u32 count = 0;
    upload_buffer_data( g_ctx.staging_buffer, &count, sizeof( u32 ) );

    const VkBufferCopy copy = {
            .srcOffset = 0,
            .dstOffset = 0,
            .size      = sizeof( u32 ) };
    vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, m_command_count_buffer.buffer, 1, &copy );
    buffer_barrier( cmd, m_command_count_buffer.buffer, m_command_count_buffer.size, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_SHADER_READ_BIT );

    vkCmdBindPipeline( cmd, VK_PIPELINE_BIND_POINT_COMPUTE, m_early_cull_pipeline.pipeline );

    CullData cull_data = {
            .draws             = m_draws_buffer.device_address,
            .cmds              = m_command_buffer.device_address,
            .meshes            = m_scene_geometry.meshes_buffer.device_address,
            .visibility        = visible_draws.device_address,
            .projection_matrix = m_camera.get_projection_matrix( ),
            .view_matrix       = m_camera.get_view_matrix( ),
            .count             = m_draws.size( ),
            .draw_count_buffer = m_command_count_buffer.device_address,
            .camera_position   = m_camera.get_position( ),
    };


    if ( !m_freeze_frustum ) {
        m_current_frustum = m_camera.get_frustum( );
    }

    cull_data.enable_lod = m_lod;

    if ( m_culling ) {
        cull_data.frustum[0] = m_current_frustum.planes[0];
        cull_data.frustum[1] = m_current_frustum.planes[1];
        cull_data.frustum[2] = m_current_frustum.planes[2];
        cull_data.frustum[3] = m_current_frustum.planes[3];
        cull_data.frustum[4] = m_current_frustum.planes[4];
        cull_data.frustum[5] = m_current_frustum.planes[5];
    }

    DrawCommandComputePushConstants pc{ .cull_data = m_cull_data[g_ctx.get_current_frame_index( )].device_address };
    upload_buffer_data( m_cull_data[g_ctx.get_current_frame_index( )], &cull_data, sizeof( CullData ) );

    vkCmdPushConstants( cmd, m_early_cull_pipeline.layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( DrawCommandComputePushConstants ), &pc );
    vkCmdDispatch( cmd, u32( m_draws.size( ) + 31 ) / 32, 1, 1 );

    vkCmdWriteTimestamp( cmd, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, frame.query_pool_timestamps, GPU_TOTAL_FIRST_CULL_STEP_END );
}

void Renderer::_late_cull( ) {
    auto& frame         = g_ctx.get_current_frame( );
    auto& cmd           = frame.cmd;
    auto& visible_draws = m_visible_draws[g_ctx.get_current_frame_index( )];

    vkCmdWriteTimestamp( cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, frame.query_pool_timestamps, GPU_TOTAL_SECOND_CULL_STEP_START );

    // Reset command count buffer to 0 (using a staging buffer)
    buffer_barrier( cmd, m_command_count_buffer.buffer, m_command_count_buffer.size, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_SHADER_READ_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT );
    u32 count = 0;
    upload_buffer_data( g_ctx.staging_buffer, &count, sizeof( u32 ) );

    const VkBufferCopy copy = {
            .srcOffset = 0,
            .dstOffset = 0,
            .size      = sizeof( u32 ) };
    vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, m_command_count_buffer.buffer, 1, &copy );
    buffer_barrier( cmd, m_command_count_buffer.buffer, m_command_count_buffer.size, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_SHADER_READ_BIT );

    if ( !m_lock_occlusion ) {
        image_barrier( cmd, frame.depth_pyramid.image, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT, VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_ASPECT_COLOR_BIT, 0, VK_REMAINING_MIP_LEVELS );
    }
    else {
        image_barrier( cmd, frame.depth_pyramid.image, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_ASPECT_COLOR_BIT, 0, VK_REMAINING_MIP_LEVELS );
    }

    // descriptor set
    std::array<DescriptorWrite, 2> writes = {
            DescriptorWrite{ .type = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, .binding = 0, .image = VkDescriptorImageInfo{ .imageView = frame.depth_pyramid.view, .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL } },
            DescriptorWrite{ .type = VK_DESCRIPTOR_TYPE_SAMPLER, .binding = 1, .image = VkDescriptorImageInfo{ .sampler = m_reduction_sampler } } };
    update_descriptor_set( m_late_cull_set[g_ctx.get_current_frame_index( )], writes.data( ), writes.size( ) );

    vkCmdBindPipeline( cmd, VK_PIPELINE_BIND_POINT_COMPUTE, m_late_cull_pipeline.pipeline );
    vkCmdBindDescriptorSets( cmd, VK_PIPELINE_BIND_POINT_COMPUTE, m_late_cull_pipeline.layout, 0, 1, &m_late_cull_set[g_ctx.get_current_frame_index( )], 0, 0 );

    // update cull data
    CullData cull_data = {
            .draws             = m_draws_buffer.device_address,
            .cmds              = m_command_buffer.device_address,
            .meshes            = m_scene_geometry.meshes_buffer.device_address,
            .visibility        = visible_draws.device_address,
            .projection_matrix = m_occlusion_perspective,
            .view_matrix       = m_occlusion_view,
            .count             = m_draws.size( ),
            .draw_count_buffer = m_command_count_buffer.device_address,
            .camera_position   = m_camera.get_position( ),
            .occlusion_data    = { frame.depth_pyramid_size, frame.depth_pyramid_size, m_camera.get_near( ), m_occlusion },
            .occlusion_data2   = { m_camera.get_projection_matrix( )[0][0], m_camera.get_projection_matrix( )[1][1], 0.0f, 0.0f },
    };

    if ( !m_freeze_frustum ) {
        m_current_frustum = m_camera.get_frustum( );
    }

    cull_data.enable_lod = m_lod;

    if ( m_culling ) {
        cull_data.frustum[0] = m_current_frustum.planes[0];
        cull_data.frustum[1] = m_current_frustum.planes[1];
        cull_data.frustum[2] = m_current_frustum.planes[2];
        cull_data.frustum[3] = m_current_frustum.planes[3];
        cull_data.frustum[4] = m_current_frustum.planes[4];
        cull_data.frustum[5] = m_current_frustum.planes[5];
    }

    upload_buffer_data( m_cull_data[g_ctx.get_current_frame_index( )], &cull_data, sizeof( CullData ) );
    DrawCommandComputePushConstants pc{
            .cull_data = m_cull_data[g_ctx.get_current_frame_index( )].device_address,
    };

    vkCmdPushConstants( cmd, m_late_cull_pipeline.layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( DrawCommandComputePushConstants ), &pc );
    vkCmdDispatch( cmd, u32( m_draws.size( ) + 31 ) / 32, 1, 1 );
    buffer_barrier( cmd, m_command_buffer.buffer, m_command_buffer.size, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT_KHR, VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT, VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT );

    vkCmdWriteTimestamp( cmd, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, frame.query_pool_timestamps, GPU_TOTAL_SECOND_CULL_STEP_END );
}

void Renderer::_construct_depth_pyramid( ) {
    auto& frame = g_ctx.get_current_frame( );
    auto& cmd   = frame.cmd;

    vkCmdBindPipeline( cmd, VK_PIPELINE_BIND_POINT_COMPUTE, m_depthpyramid_pipeline.pipeline );

    // transition depth image to be sampled
    image_barrier( cmd, frame.depth.image, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_ASPECT_DEPTH_BIT );

    // transition pyramid image to storage
    image_barrier( cmd, frame.depth_pyramid.image, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_IMAGE_LAYOUT_GENERAL );

    // Update descriptor sets
    auto& set = m_depthpyramid_sets[g_ctx.get_current_frame_index( )];
    {
        VkDescriptorImageInfo depth_info = {
                .imageView = frame.depth.view, .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };

        VkDescriptorImageInfo sampler_info = { .sampler = m_reduction_sampler };

        std::vector<VkDescriptorImageInfo> pyramid_info;
        for ( u32 i = 0; i < frame.depth_pyramid_levels; i++ ) {
            pyramid_info.push_back(
                    { .imageView = frame.depth_pyramid_mips[i], .imageLayout = VK_IMAGE_LAYOUT_GENERAL } );
        }

        std::array<VkWriteDescriptorSet, 4> descriptor_writes = {
                VkWriteDescriptorSet{
                        .sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                        .pNext           = nullptr,
                        .dstSet          = set,
                        .dstBinding      = 0,
                        .dstArrayElement = 0,
                        .descriptorCount = 1,
                        .descriptorType  = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,
                        .pImageInfo      = &depth_info },
                VkWriteDescriptorSet{
                        .sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                        .pNext           = nullptr,
                        .dstSet          = set,
                        .dstBinding      = 1,
                        .dstArrayElement = 0,
                        .descriptorCount = frame.depth_pyramid_levels,
                        .descriptorType  = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,
                        .pImageInfo      = pyramid_info.data( ) },
                VkWriteDescriptorSet{
                        .sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                        .pNext           = nullptr,
                        .dstSet          = set,
                        .dstBinding      = 2,
                        .dstArrayElement = 0,
                        .descriptorCount = frame.depth_pyramid_levels,
                        .descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                        .pImageInfo      = pyramid_info.data( ) },
                VkWriteDescriptorSet{
                        .sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                        .pNext           = nullptr,
                        .dstSet          = set,
                        .dstBinding      = 3,
                        .dstArrayElement = 0,
                        .descriptorCount = 1,
                        .descriptorType  = VK_DESCRIPTOR_TYPE_SAMPLER,
                        .pImageInfo      = &sampler_info },
        };
        vkUpdateDescriptorSets( g_ctx.device, descriptor_writes.size( ), descriptor_writes.data( ), 0, nullptr );
    }

    for ( u32 i = 0; i < frame.depth_pyramid_levels; i++ ) {
        DepthPyramidPushConstants pc = {
                .mip_size = { frame.depth_pyramid_size >> i, frame.depth_pyramid_size >> i, i } };

        vkCmdPushConstants( cmd, m_depthpyramid_pipeline.layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( DepthPyramidPushConstants ), &pc );
        vkCmdBindDescriptorSets( cmd, VK_PIPELINE_BIND_POINT_COMPUTE, m_depthpyramid_pipeline.layout, 0, 1, &set, 0, nullptr );
        vkCmdDispatch( cmd, ( pc.mip_size.x + 31 ) / 32, ( pc.mip_size.y + 31 ) / 32, 1 );

        // depth image is already transitioned at the start of the function
        if ( i != 0 ) {
            // this mip was previously being written to, now its gonna be read from
            image_barrier( cmd, frame.depth_pyramid.image, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_ASPECT_COLOR_BIT, i - 1, 1 );
        }

        // image barrier for mip i to be written to
        image_barrier( cmd, frame.depth_pyramid.image, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, 0, VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_ASPECT_COLOR_BIT, i, 1 );
    }
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
            else if ( event.key.keysym.scancode == SDL_SCANCODE_L ) {
                m_lod = !m_lod;
            }
            else if ( event.key.keysym.scancode == SDL_SCANCODE_O ) {
                m_occlusion = !m_occlusion;
            }
            else if ( event.key.keysym.scancode == SDL_SCANCODE_P ) {
                m_lock_occlusion = !m_lock_occlusion;
            }
            else if ( event.key.keysym.scancode == SDL_SCANCODE_SPACE ) {
                auto& draw = m_draws.at( 0 );

                glm::mat4 model = glm::translate( glm::mat4( 1.0f ), m_camera.get_position( ) );
                model           = glm::scale( model, glm::vec3( 2.0f ) );
                draw.model      = model;

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
        m_camera.set_position( current_pos + glm::normalize( movement ) * move_speed * ( f32 )m_delta_time );
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

            // insert the vertices into the global scene. these will be the same for all LODs and meshlets
            u32 vertex_offset = scene.vertices.size( );
            scene.vertices.insert( scene.vertices.end( ), vertices.begin( ), vertices.end( ) );

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
            };

            f32 threshold = 1;
            for ( u32 i = 0; i < 6; i++ ) {
                Lod& lod = mesh.lods[i];

                std::vector<u32> lod_indices = indices;

                // LOD 0 its the original mesh so dont simplify it
                if ( i != 0 ) {
                    threshold *= 0.5f;
                    f32  error              = 0.001f * std::pow( 2.0f, i );
                    u32  target_index_count = indices.size( ) * threshold;
                    auto res                = meshopt_simplify( lod_indices.data( ), lod_indices.data( ), lod_indices.size( ), &vertices[0].vx, vertices.size( ), sizeof( Vertex ), target_index_count, error );
                    lod_indices.resize( res );
                }

                // insert new indices and vertices into the global scene
                scene.indices.insert( scene.indices.end( ), lod_indices.begin( ), lod_indices.end( ) );

                // generate meshlets and get offset into global scene
                u32 meshlet_index = scene.meshlets.size( );
                build_meshlets( vertices, lod_indices, scene );
                u32 meshlet_count = scene.meshlets.size( ) - meshlet_index;

                lod.meshlet_count = meshlet_count;
                lod.meshlet_index = meshlet_index;
            }

            scene.meshes.emplace_back( mesh );

            return scene.meshes.size( ) - 1;
        }
    }

    return -1;
}