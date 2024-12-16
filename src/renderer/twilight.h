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

#include <optional>
#include <renderer/r_shaders.h>
#include <types.h>
#include <vulkan/vulkan_core.h>
#include "glm/ext/vector_uint2.hpp"
#include "r_camera.h"
#include "r_resources.h"

namespace tl {

    struct CullData {
        VkDeviceAddress draws;
        VkDeviceAddress cmds;
        VkDeviceAddress meshes;
        VkDeviceAddress visibility;
        glm::mat4       projection_matrix;
        glm::mat4       view_matrix;
        u64             count;
        VkDeviceAddress draw_count_buffer;
        glm::vec3       camera_position;
        u32             enable_lod;
        glm::vec4       frustum[6];
        glm::vec4       occlusion_data;  // width, height, znear, enabled
        glm::vec4       occlusion_data2; // p00, p11
    };

    struct DrawCommandComputePushConstants {
        VkDeviceAddress cull_data;
    };

    struct DrawMeshTaskCommand {
        u32 x;
        u32 y;
        u32 z;
        u32 draw_id;
        u32 lod_id;
        u32 pad;
    };

    struct ScenePushConstants {
        glm::mat4 view;
        glm::mat4 projection;
        glm::vec4 camera_position;
        u64       draws_buffer;
        u64       meshes;
        u64       meshlets_buffer;
        u64       meshlets_data_buffer;
        u64       vertex_buffer;
        u64       draw_cmds;
    };

    struct DepthPyramidPushConstants {
        glm::uvec3 mip_size;
    };

    struct Vertex {
        f32 vx, vy, vz;
        f32 nx, ny, nz;
    };

    const size_t max_vertices  = 64;
    const size_t max_triangles = 124;
    struct Meshlet {
        u64 data_offset; // SceneGeometry::meshlet_data -> [data_offset ... data_offset + vertex_count[ = vertex indices (use index + mesh.vertex_offset to index into vertices_buffer).
                         // After that until index_count, indices
                         //
                         // u32 vindex = meshlet_data[data_offset + i] + mesh.vertex_offset;
                         // Vertex v = vertices[index];

        u32 vertex_count;
        u32 triangle_count; // triangle count (to get index count do x3)

        // backface culling
        f32 cone_apex[3];
        f32 cone_axis[3];
        f32 cone_cutoff;
    };

    struct Lod {
        u32 meshlet_index; // first meshlet index into SceneGeometry::meshlets;
        u32 meshlet_count;
    };

    struct Mesh {
        glm::vec3 center;
        f32       radius;

        u32 vertex_offset;
        u32 pad;

        Lod lods[6];
    };

    struct Draw {
        glm::mat4 model;
        u64       mesh;
        u64       pad;
    };

    struct SceneGeometry {
        std::vector<Vertex>  vertices;
        std::vector<u32>     indices;
        std::vector<Meshlet> meshlets;
        std::vector<u32>     meshlet_data; // Contains the meshlet data (vertex indices & triangles)
        std::vector<Mesh>    meshes;

        Buffer vertices_buffer;
        // Buffer indices_buffer;
        Buffer meshlets_buffer;
        Buffer meshlet_data_buffer;
        Buffer meshes_buffer;
    };

    // Timestamp values
#define GPU_TOTAL_START 0
#define GPU_TOTAL_END GPU_TOTAL_START + 1
#define GPU_TOTAL_FIRST_CULL_STEP_START 2
#define GPU_TOTAL_FIRST_CULL_STEP_END GPU_TOTAL_FIRST_CULL_STEP_START + 1
#define GPU_TOTAL_SECOND_CULL_STEP_START 4
#define GPU_TOTAL_SECOND_CULL_STEP_END GPU_TOTAL_SECOND_CULL_STEP_START + 1

    class Renderer {
    public:
        void Initialize( int count );
        void Shutdown( );

        void Run( );

        u32 width  = 1920;
        u32 height = 1080;

    private:
        void process_events( );
        void _issue_draw_calls( u32 swapchain_image_idx, bool clear_color = true, bool clear_depth = true );
        void _upload_scene_geometry( );
        void _early_cull( );
        void _late_cull( );
        void _construct_depth_pyramid( );

        SDL_Window* m_window = { };
        bool        m_quit   = false;

        bool      m_occlusion             = true;
        bool      m_lock_occlusion        = false;
        glm::mat4 m_occlusion_view        = { };
        glm::mat4 m_occlusion_perspective = { };
        bool      m_culling               = true;
        bool      m_freeze_frustum        = false;
        bool      m_lod                   = false;
        Frustum   m_current_frustum       = { };
        u64       m_frame_triangles       = 0; // how many triangles were drawn this frame

        Pipeline m_mesh_pipeline;
        Pipeline m_early_cull_pipeline;
        Pipeline m_late_cull_pipeline;
        Pipeline m_depthpyramid_pipeline;

        VkSampler m_linear_sampler    = VK_NULL_HANDLE;
        VkSampler m_reduction_sampler = VK_NULL_HANDLE; // Sampler used for MAX reduction on depth pyramid creation

        VkDescriptorSetLayout        m_late_cull_descriptor_layout = VK_NULL_HANDLE;
        std::vector<VkDescriptorSet> m_late_cull_set;

        VkDescriptorSetLayout        m_depthpyramid_descriptor_layout;
        std::vector<VkDescriptorSet> m_depthpyramid_sets;

        u64                   m_draws_count          = 1'000'000;
        Buffer                m_command_buffer       = { };
        Buffer                m_draws_buffer         = { };
        std::vector<Draw>     m_draws                = { };
        Buffer                m_command_count_buffer = { };
        std::array<Buffer, 2> m_cull_data            = { };
        std::array<Buffer, 3> m_visible_draws        = { }; // NOTE: 1 = visible last frame | 0 = NOT visible last frame
        // 0 -> 1 frame
        // 1 -> another frame
        // 2 -> buffer frame used to copy between them

        f64           m_last_frame_time = 0.0;
        f64           m_delta_time      = 0.0;
        Camera        m_camera;
        float         move_speed       = 15.0f;
        SceneGeometry m_scene_geometry = { };
    };

    void build_meshlets( const std::vector<Vertex>& vertices, const std::vector<u32>& indices, SceneGeometry& scene );
    u32  load_mesh_from_file( const std::string& gltf_path, const std::string& mesh_name, SceneGeometry& scene );
} // namespace tl