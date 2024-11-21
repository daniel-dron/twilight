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
#include "r_camera.h"
#include "r_resources.h"

namespace tl {
    struct ScenePushConstants {
        glm::mat4 view;
        glm::mat4 projection;
        glm::mat4 model;
        glm::vec4 camera_position;
        u64       vertex_buffer;
        u64       meshlets_buffer;
        u64       meshlet_vertices;
        u64       meshlet_triangles;
        u32       meshlet_count;
    };

    struct Vertex {
        f32 vx, vy, vz;
    };

    const size_t max_vertices  = 64;
    const size_t max_triangles = 124;
    struct Meshlet {
        u32 vertex_offset;
        u32 triangle_offset;
        u32 vertex_count;
        u32 triangle_count;

        // backface culling
        f32 cone_apex[3];
        f32 cone_axis[3];
        f32 cone_cutoff;
    };

    struct Mesh {
        std::vector<Vertex> vertices;
        std::vector<u32>    indices;
        Buffer              vertex_buffer;
        Buffer              index_buffer;

        std::vector<Meshlet> meshlets;
        Buffer               meshlets_buffer;
        Buffer               meshlets_vertices;
        Buffer               meshlets_triangles;

        glm::vec3 min;
        glm::vec3 max;
    };

    struct MeshDraw {
        Mesh&     mesh;
        glm::mat4 model;
    };

    struct MeshDrawPushConstants {
        glm::mat4 model;
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

        SDL_Window* m_window = { };
        bool        m_quit   = false;

        u64 m_frame_triangles = 0; // how many triangles were drawn this frame

        bool m_use_mesh_pipeline = true;

        Pipeline m_pipeline;
        Pipeline m_mesh_pipeline;

        Camera                m_camera;
        // float                 move_speed = 100.0f;
        float                 move_speed = 0.005f;
        Mesh                  m_mesh;
        std::vector<MeshDraw> m_draws;
    };

    void                build_meshlets( Mesh& mesh );
    std::optional<Mesh> load_mesh_from_file( const std::string& gltf_path, const std::string& mesh_name );
    void                destroy_mesh( Mesh& mesh );

} // namespace tl