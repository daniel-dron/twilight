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

    struct DrawCommandComputePushConstants {
        VkDeviceAddress draws;
        VkDeviceAddress cmds;
        VkDeviceAddress meshes;
        u64             count;
        VkDeviceAddress draw_count_buffer;
        glm::vec3       camera_position;
        u32             enable_lod;
        glm::vec4       frustum[6];
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

    class Renderer {
    public:
        void Initialize( );
        void Shutdown( );

        void Run( );

        u32 width  = 1920;
        u32 height = 1080;

    private:
        void tick( u32 swapchain_image_idx );
        void process_events( );
        void _upload_scene_geometry( );
        void _construct_depth_pyramid( );

        SDL_Window* m_window = { };
        bool        m_quit   = false;

        i32     m_display_depth   = -1;
        bool    m_culling         = true;
        bool    m_freeze_frustum  = false;
        bool    m_lod             = true;
        Frustum m_current_frustum = { };
        u64     m_frame_triangles = 0; // how many triangles were drawn this frame

        Pipeline m_mesh_pipeline;
        Pipeline m_drawcmd_pipeline;
        Pipeline m_depthpyramid_pipeline;

        VkSampler m_linear_sampler    = VK_NULL_HANDLE;
        VkSampler m_reduction_sampler = VK_NULL_HANDLE; // Sampler used for MAX reduction on depth pyramid creation

        VkDescriptorSetLayout        m_depthpyramid_descriptor_layout;
        std::vector<VkDescriptorSet> m_depthpyramid_sets;

        u64               m_draws_count          = 100'000;
        Buffer            m_command_buffer       = { };
        Buffer            m_draws_buffer         = { };
        std::vector<Draw> m_draws                = { };
        Buffer            m_command_count_buffer = { };

        Camera        m_camera;
        float         move_speed       = 0.5f;
        SceneGeometry m_scene_geometry = { };
    };

    void build_meshlets( const std::vector<Vertex>& vertices, const std::vector<u32>& indices, SceneGeometry& scene );
    u32  load_mesh_from_file( const std::string& gltf_path, const std::string& mesh_name, SceneGeometry& scene );
} // namespace tl