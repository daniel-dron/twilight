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

#include <renderer/r_shaders.h>
#include <types.h>
#include <vulkan/vulkan_core.h>
#include "r_camera.h"
#include "r_resources.h"
#include "r_scene.h"

namespace tl {

    struct CullData {
        VkDeviceAddress draws;
        VkDeviceAddress tasks;    // Buffer of tasks for mesh pipeline (task/amplification stage)
        VkDeviceAddress commands; // Buffer of indirect draw arguments for vertex pipeline
        VkDeviceAddress meshes;
        VkDeviceAddress visibility;
        u64             pad;
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
        f64             time;
        uint32_t        use_task;       // Wether to write to tasks or commands buffer in CullData
    };

    struct DrawMeshTaskCommand {
        u32 x;
        u32 y;
        u32 z;
        u32 draw_id;
        u32 lod_id;
        u32 pad;
    };

    struct DrawIndexedIndirectCommand {
        uint32_t indexCount;
        uint32_t instanceCount;
        uint32_t firstIndex;
        int32_t  vertexOffset;
        uint32_t firstInstance;

        uint32_t draw_id;
        uint32_t lod_id;
        float    pad;
        float    pad1;
        float    pad2;
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
        u64       tasks_buffer;
        u64       commands_buffer;
    };

    struct DepthPyramidPushConstants {
        glm::uvec3 mip_size;
    };


    // Timestamp values
#define GPU_TOTAL_START 0
#define GPU_TOTAL_END GPU_TOTAL_START + 1
#define GPU_TOTAL_FIRST_CULL_STEP_START 2
#define GPU_TOTAL_FIRST_CULL_STEP_END GPU_TOTAL_FIRST_CULL_STEP_START + 1
#define GPU_TOTAL_SECOND_CULL_STEP_START 4
#define GPU_TOTAL_SECOND_CULL_STEP_END GPU_TOTAL_SECOND_CULL_STEP_START + 1
#define GPU_TOTAL_UPDATE_SCENE_START 6
#define GPU_TOTAL_UPDATE_SCENE_END GPU_TOTAL_UPDATE_SCENE_START + 1

    class Renderer {
    public:
        void Initialize( int count );
        void Shutdown( );

        void Run( );

        u32 width  = 1920;
        u32 height = 1080;

    private:
        void process_events( );
        void _issue_draw_calls( u32 swapchain_image_idx, Pipeline& pipeline, bool clear_color = true, bool clear_depth = true );
        void _upload_scene_geometry( );
        void _early_cull( );
        void _late_cull( );
        void _construct_depth_pyramid( );

        SDL_Window* m_window = { };
        bool        m_quit   = false;

        bool      m_use_mesh_shaders      = false;
        bool      m_draw_aabbs            = false;
        bool      m_visualize_overdraw    = false;
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
        Pipeline m_vertex_pipeline;
        Pipeline m_aabb_pipeline;
        Pipeline m_early_cull_pipeline;
        Pipeline m_late_cull_pipeline;
        Pipeline m_depthpyramid_pipeline;

        Pipeline                     m_overdraw_accumulation_pipeline  = { };
        Pipeline                     m_overdraw_visualization_pipeline = { };
        std::vector<VkDescriptorSet> m_overdraw_sets                   = { };

        VkSampler m_linear_sampler    = VK_NULL_HANDLE;
        VkSampler m_reduction_sampler = VK_NULL_HANDLE; // Sampler used for MAX reduction on depth pyramid creation

        std::vector<VkDescriptorSet> m_late_cull_set;
        std::vector<VkDescriptorSet> m_depthpyramid_sets;

        u64    m_draws_count           = 100'000;
        Buffer m_task_command_buffer   = { };
        Buffer m_draw_command_buffer   = { };
        Buffer m_command_count_buffer  = { };
        Buffer m_cull_data             = { };
        Buffer m_visible_draws         = { }; // NOTE: 1 = visible last frame | 0 = NOT visible last frame
        Buffer m_visible_draws_staging = { }; //  buffer frame used to copy between them

        f64           m_last_frame_time = 0.0;
        f64           m_delta_time      = 0.0;
        Camera        m_camera;
        float         move_speed       = 15.0f;
        SceneGeometry m_scene_geometry = { };

        GPUScene m_gpu_scene = { };
    };

    void build_meshlets( const std::vector<Vertex>& vertices, const std::vector<u32>& indices, SceneGeometry& scene );
    u32  load_mesh_from_file( const std::string& gltf_path, const std::string& mesh_name, SceneGeometry& scene );
} // namespace tl