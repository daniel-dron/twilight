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
#include "r_resources.h"

namespace tl {
    struct Camera {
        glm::vec3 position;
        glm::quat orientation;
        float     fov;
        float     near;
        float     far;
    };

    struct ScenePushConstants {
        glm::mat4 view;
        glm::mat4 projection;
        u64       vertex_buffer;
    };

    struct Vertex {
        f32 vx, vy, vz;
    };

    struct Mesh {
        std::vector<Vertex> vertices;
        std::vector<u32>    indices;
        Buffer              vertex_buffer;
        Buffer              index_buffer;
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

        Pipeline m_pipeline;
        Camera   m_camera;
        Mesh     m_mesh;
        Buffer   m_staging_buffer; // global staging buffer (100MB)
    };

    std::optional<Mesh> load_mesh_from_file( const std::string& gltf_path, const std::string& mesh_name );

} // namespace tl