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

    m_command_buffer = create_buffer( sizeof( VkDrawMeshTasksIndirectCommandEXT ) * 2000, VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY );
    m_draws_buffer   = create_buffer( sizeof( Draw ) * 2000, VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true );
    m_meshes_buffer  = create_buffer( sizeof( Mesh ) * 100, VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true );

    m_mesh_assets.emplace_back( load_mesh_from_file( "../../assets/lucy/lucy.gltf", "Lucy_3M_O10" ).value( ) );
    m_mesh_assets.emplace_back( load_mesh_from_file( "../../assets/teapot/teapot.gltf", "Teapot" ).value( ) );
    m_mesh_assets.emplace_back( load_mesh_from_file( "../../assets/cube/cube.gltf", "Cube.001" ).value( ) );

    for ( auto& mesh_asset : m_mesh_assets ) {
        Mesh mesh{
                .vertex_buffer     = mesh_asset.vertex_buffer.device_address,
                .meshlet_buffer    = mesh_asset.meshlets_buffer.device_address,
                .meshlet_vertices  = mesh_asset.meshlets_vertices.device_address,
                .meshlet_triangles = mesh_asset.meshlets_triangles.device_address,
                .meshlet_count     = mesh_asset.meshlets.size( ) };
        m_meshes.emplace_back( mesh );
    }

    {
        auto& cmd = g_ctx.global_cmd;
        VKCALL( vkResetFences( g_ctx.device, 1, &g_ctx.global_fence ) );
        VKCALL( vkResetCommandBuffer( cmd, 0 ) );
        begin_command( cmd, VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT );

        upload_buffer_data( g_ctx.staging_buffer, m_meshes.data( ), sizeof( Mesh ) * m_meshes.size( ) );

        const VkBufferCopy copy = {
                .srcOffset = 0,
                .dstOffset = 0,
                .size      = sizeof( Mesh ) * m_meshes.size( ) };
        vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, m_meshes_buffer.buffer, 1, &copy );

        VKCALL( vkEndCommandBuffer( cmd ) );
        submit_command( cmd, g_ctx.graphics_queue, g_ctx.global_fence );
        VKCALL( vkWaitForFences( g_ctx.device, 1, &g_ctx.global_fence, true, UINT64_MAX ) );
    }

    u64 count  = 2000;
    f32 radius = 50.0f;
    m_draws.reserve( count );

    std::random_device                    rd;
    std::mt19937                          gen( rd( ) );
    std::uniform_real_distribution<float> posDist( -radius, radius );
    std::uniform_real_distribution<float> rotDist( 0.0f, 360.0f );
    std::uniform_real_distribution<float> scaleDist( 0.8f, 1.2f );

    for ( int i = 0; i < count; ++i ) {
        u64 mesh_id = rand( ) % m_mesh_assets.size( );

        glm::vec3 position(
                posDist( gen ),
                posDist( gen ),
                posDist( gen ) );

        float rotX = glm::radians( rotDist( gen ) );
        float rotY = glm::radians( rotDist( gen ) );
        float rotZ = glm::radians( rotDist( gen ) );

        float scale = scaleDist( gen );
        if ( mesh_id == 0 ) {
            scale /= 100.0f;
        }

        glm::mat4 model = glm::mat4( 1.0f );
        model           = glm::translate( model, position );
        model           = glm::rotate( model, rotX, glm::vec3( 1.0f, 0.0f, 0.0f ) );
        model           = glm::rotate( model, rotY, glm::vec3( 0.0f, 1.0f, 0.0f ) );
        model           = glm::rotate( model, rotZ, glm::vec3( 0.0f, 0.0f, 1.0f ) );
        model           = glm::scale( model, glm::vec3( scale ) );

        m_draws.emplace_back( Draw{ model, mesh_id } );

        VkDrawMeshTasksIndirectCommandEXT command = {
                .groupCountX = u32( m_mesh_assets.at( mesh_id ).meshlets.size( ) + 31 ) / 32,
                .groupCountY = 1,
                .groupCountZ = 1 };
        m_commands.emplace_back( command );
    }

    {
        auto& cmd = g_ctx.global_cmd;
        VKCALL( vkResetFences( g_ctx.device, 1, &g_ctx.global_fence ) );
        VKCALL( vkResetCommandBuffer( cmd, 0 ) );
        begin_command( cmd, VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT );

        upload_buffer_data( g_ctx.staging_buffer, m_commands.data( ), sizeof( VkDrawMeshTasksIndirectCommandEXT ) * m_commands.size( ) );
        upload_buffer_data( g_ctx.staging_buffer, m_draws.data( ), sizeof( Draw ) * m_draws.size( ), sizeof( VkDrawMeshTasksIndirectCommandEXT ) * m_commands.size( ) );

        const VkBufferCopy copy = {
                .srcOffset = 0,
                .dstOffset = 0,
                .size      = sizeof( VkDrawMeshTasksIndirectCommandEXT ) * m_commands.size( ) };
        vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, m_command_buffer.buffer, 1, &copy );

        const VkBufferCopy draws_copy = {
                .srcOffset = sizeof( VkDrawMeshTasksIndirectCommandEXT ) * m_commands.size( ),
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
    destroy_buffer( m_meshes_buffer );

    for ( auto& mesh : m_mesh_assets ) {
        destroy_mesh( mesh );
    }

    m_mesh_pipeline.shutdown( );
    g_ctx.shutdown( );
    SDL_DestroyWindow( m_window );
}

void Renderer::Run( ) {
    f64 avg_cpu_time = 0;
    f64 avg_gpu_time = 0;

    for ( auto& draw : m_draws ) {
        auto mesh = m_mesh_assets.at( draw.mesh );
        for ( auto& meshlet : mesh.meshlets ) {
            m_frame_triangles += meshlet.triangle_count;
        }
    }

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
        vkGetQueryPoolResults( g_ctx.device, frame.query_pool_timestamps, 0, u32( frame.gpu_timestamps.size( ) ), frame.gpu_timestamps.size( ) * sizeof( u64 ), frame.gpu_timestamps.data( ), sizeof( u64 ), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT );

        g_ctx.current_frame++;
        auto cpu_time_end = get_time( );

        avg_cpu_time = avg_cpu_time * 0.95 + ( ( cpu_time_end - cpu_time_start ) * 1000.0f ) * 0.05;
        avg_gpu_time = avg_gpu_time * 0.95 + ( g_ctx.get_query_time_in_ms( frame.gpu_timestamps[0], frame.gpu_timestamps[1] ) ) * 0.05f;

        f64 triangles_per_sec = f64( m_frame_triangles ) / f64( avg_cpu_time * 1e-3 );

        auto title = std::format( "cpu: {:.3f}; gpu: {:.3f}; triangles {:.2f}M; {:.1f}B tri/sec;", avg_cpu_time, avg_gpu_time, f64( m_frame_triangles ) * 1e-6, triangles_per_sec * 1e-9 );
        SDL_SetWindowTitle( m_window, title.c_str( ) );
    }
}

void Renderer::tick( u32 swapchain_image_idx ) {
    auto& frame = g_ctx.get_current_frame( );
    auto& cmd   = frame.cmd;

    VkClearValue              clear_color{ 0.1f, 0.05f, 0.1f, 1.0f };
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
            .view            = m_camera.get_view_matrix( ),
            .projection      = m_camera.get_projection_matrix( ),
            .camera_position = glm::vec4( m_camera.get_position( ), 1.0f ),
            .draws_buffer    = m_draws_buffer.device_address,
            .meshes          = m_meshes_buffer.device_address,
    };

    vkCmdPushConstants( cmd, m_mesh_pipeline.layout, VK_SHADER_STAGE_ALL_GRAPHICS | VK_SHADER_STAGE_MESH_BIT_EXT | VK_SHADER_STAGE_TASK_BIT_EXT, 0, sizeof( ScenePushConstants ), &pc );
    vkCmdDrawMeshTasksIndirectEXT( cmd, m_command_buffer.buffer, 0, m_commands.size( ), sizeof( VkDrawMeshTasksIndirectCommandEXT ) );


    vkCmdEndRendering( cmd );
}

void tl::build_meshlets( MeshAsset& mesh ) {
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

std::optional<MeshAsset> tl::load_mesh_from_file( const std::string& gltf_path, const std::string& mesh_name ) {
    assert( !gltf_path.empty( ) );
    assert( !mesh_name.empty( ) );
    assert( std::filesystem::exists( gltf_path ) );

    Assimp::Importer importer;
    const auto       aiScene = importer.ReadFile( gltf_path, aiProcess_Triangulate | aiProcess_GenBoundingBoxes | aiProcess_FlipWindingOrder );

    for ( size_t i = 0; i < aiScene->mNumMeshes; i++ ) {
        if ( std::string( aiScene->mMeshes[i]->mName.C_Str( ) ) == mesh_name ) {
            MeshAsset mesh;
            mesh.vertices.clear( );
            mesh.indices.clear( );

            auto ai_mesh = aiScene->mMeshes[i];

            mesh.vertices.reserve( ai_mesh->mNumVertices );
            for ( u32 i = 0; i < ai_mesh->mNumVertices; i++ ) {
                auto vertex  = ai_mesh->mVertices[i];
                auto normals = ai_mesh->mNormals[i];

                mesh.vertices.push_back( { vertex.x, vertex.y, vertex.z, normals.x, normals.y, normals.z } );
            }

            mesh.indices.reserve( ai_mesh->mNumFaces * 3 );
            for ( u32 i = 0; i < ai_mesh->mNumFaces; i++ ) {
                auto& face = ai_mesh->mFaces[i];
                mesh.indices.emplace_back( face.mIndices[0] );
                mesh.indices.emplace_back( face.mIndices[1] );
                mesh.indices.emplace_back( face.mIndices[2] );
            }

            mesh.min = glm::vec3( ai_mesh->mAABB.mMin.x, ai_mesh->mAABB.mMin.y, ai_mesh->mAABB.mMin.z );
            mesh.max = glm::vec3( ai_mesh->mAABB.mMax.x, ai_mesh->mAABB.mMax.y, ai_mesh->mAABB.mMax.z );

            mesh.vertex_buffer = create_buffer( mesh.vertices.size( ) * sizeof( Vertex ), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true, false );
            mesh.index_buffer  = create_buffer( mesh.indices.size( ) * sizeof( u32 ), VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY );

            build_meshlets( mesh );
            mesh.meshlets_buffer = create_buffer( mesh.meshlets.size( ) * sizeof( Meshlet ), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 0, VMA_MEMORY_USAGE_GPU_ONLY, true, false );

            // upload mesh
            upload_buffer_data( g_ctx.staging_buffer, mesh.vertices.data( ), mesh.vertex_buffer.size );
            upload_buffer_data( g_ctx.staging_buffer, mesh.indices.data( ), mesh.index_buffer.size, mesh.vertex_buffer.size ); // Upload to the same staging buffer after the vertex data;
            upload_buffer_data( g_ctx.staging_buffer, mesh.meshlets.data( ), mesh.meshlets_buffer.size, mesh.vertex_buffer.size + mesh.index_buffer.size );

            {
                auto& cmd = g_ctx.global_cmd;
                VKCALL( vkResetFences( g_ctx.device, 1, &g_ctx.global_fence ) );
                VKCALL( vkResetCommandBuffer( cmd, 0 ) );
                begin_command( cmd, VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT );

                // Copy vertex and index buffer
                const VkBufferCopy vertex_copy = {
                        .srcOffset = 0,
                        .dstOffset = 0,
                        .size      = mesh.vertex_buffer.size };
                const VkBufferCopy index_copy = {
                        .srcOffset = mesh.vertex_buffer.size,
                        .dstOffset = 0,
                        .size      = mesh.index_buffer.size };
                vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, mesh.vertex_buffer.buffer, 1, &vertex_copy );
                vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, mesh.index_buffer.buffer, 1, &index_copy );

                // Copy meshlets
                const VkBufferCopy meshlets_copy = {
                        .srcOffset = mesh.vertex_buffer.size + mesh.index_buffer.size,
                        .dstOffset = 0,
                        .size      = mesh.meshlets_buffer.size };
                vkCmdCopyBuffer( cmd, g_ctx.staging_buffer.buffer, mesh.meshlets_buffer.buffer, 1, &meshlets_copy );

                VKCALL( vkEndCommandBuffer( cmd ) );
                submit_command( cmd, g_ctx.graphics_queue, g_ctx.global_fence );
                VKCALL( vkWaitForFences( g_ctx.device, 1, &g_ctx.global_fence, true, UINT64_MAX ) );
            }

            return mesh;
        }
    }

    return std::nullopt;
}

void tl::destroy_mesh( MeshAsset& mesh ) {
    destroy_buffer( mesh.vertex_buffer );
    destroy_buffer( mesh.index_buffer );
    destroy_buffer( mesh.meshlets_buffer );
    destroy_buffer( mesh.meshlets_vertices );
    destroy_buffer( mesh.meshlets_triangles );
}