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

#include "twilight.h"

using namespace tl;

void Renderer::Initialize( ) {
    // Initialize SDL
    SDL_Init( SDL_INIT_VIDEO );
    SDL_WindowFlags flags = ( SDL_WindowFlags )( SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE );

    m_window = SDL_CreateWindow( "Twilight", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, flags );

    m_vulkan_context.initialize( WIDTH, HEIGHT, "Twilight", m_window );
}

void Renderer::Shutdown( ) {
    m_vulkan_context.shutdown( );

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
    }
}