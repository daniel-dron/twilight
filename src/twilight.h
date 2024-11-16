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

#include <types.h>
#include "renderer/r_context.h"

namespace tl {

    class Renderer {
    public:
        void Initialize( );
        void Shutdown( );

        void Run( );

        static const u32 WIDTH  = 1920;
        static const u32 HEIGHT = 1080;

    private:
        void process_events( );

        SDL_Window* m_window = { };
        bool        m_quit   = false;

        Context m_vulkan_context = { };
    };

} // namespace tl