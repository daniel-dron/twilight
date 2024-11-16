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

#include <SDL2/SDL_video.h>
#include <pch.h>
#include "SDL.h"
#include "SDL_events.h"
#include "renderer/r_context.h"

#include "twilight.h"

using namespace tl;

void Renderer::Initialize( ) {
    // Initialize SDL
    SDL_Init( SDL_INIT_VIDEO );
    SDL_WindowFlags flags = ( SDL_WindowFlags )( SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE );

    m_window = SDL_CreateWindow( "Twilight", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, flags );

    m_ctx.initialize( width, height, "Twilight", m_window );
}

void Renderer::Shutdown( ) {
    m_ctx.shutdown( );

    SDL_DestroyWindow( m_window );
}

void Renderer::Run( ) {
    while ( !m_quit ) {
        process_events( );
    }
}

void Renderer::process_events( ) {
    SDL_Event event;
    while ( SDL_PollEvent( &event ) != 0 ) {
        if ( event.type == SDL_QUIT ) {
            m_quit = true;
        }

        if ( event.type == SDL_WINDOWEVENT ) {
            if ( event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED && event.window.data1 > 0 && event.window.data2 > 0 ) {
                this->width  = event.window.data1;
                this->height = event.window.data2;
                m_ctx.swapchain.resize( event.window.data1, event.window.data2, m_ctx.chosen_gpu, m_ctx.device, m_ctx.surface );
            }
        }
    }
}