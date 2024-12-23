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

namespace tl {
    struct Frustum {
        glm::vec4 planes[6];
    };

    class Camera {
    public:
        Camera( ) {};
        Camera( const glm::vec3& position, float yaw, float pitch, float width, float height );

        void set_aspect_ratio( float width, float height );
        void rotate( float deltaYaw, float deltaPitch, float deltaRoll = 0.0f );

        const glm::vec3& get_front( ) const;
        const glm::vec3& get_right( ) const;
        const glm::vec3& get_position( ) const;
        void             set_position( const glm::vec3& newPosition );

        const glm::mat4& get_view_matrix( );
        const glm::mat4& get_projection_matrix( );
        Frustum          get_frustum( );

        float get_near( ) { return m_near_plane; };

    private:
        void _update_vectors( );
        void _update_matrices( );
        void _update_frustum( );

        glm::vec4 m_frustum[6];

        glm::vec3 m_position = { 0.0f, 0.0f, 0.0f };
        glm::vec3 m_front    = GLOBAL_FRONT;
        glm::vec3 m_right    = GLOBAL_RIGHT;
        glm::vec3 m_up       = GLOBAL_UP;
        glm::vec3 m_world_up = GLOBAL_UP;

        glm::mat4 m_view_matrix       = glm::mat4( 1.0f );
        glm::mat4 m_projection_matrix = glm::mat4( 1.0f );

        float m_yaw   = 0.0f;
        float m_roll  = 0.0f;
        float m_pitch = 0.0f;

        float m_min_pitch = -89.0f;
        float m_max_pitch = 89.0f;

        float m_fov     = 90.0f;
        float m_max_fov = 130.0f;
        float m_min_ov  = 20.0f;

        float m_aspect_ratio = 0.0f;

        float m_near_plane = 0.1f;
        float m_far_plane  = 2000.0f;

        bool m_dirtyMatrices = true;
    };
}; // namespace tl