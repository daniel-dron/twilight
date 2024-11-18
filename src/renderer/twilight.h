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
    };

} // namespace tl