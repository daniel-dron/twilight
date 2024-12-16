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
#include <pch.h>
#include "glm/ext/matrix_clip_space.hpp"

#include "r_camera.h"

namespace tl {
    Camera::Camera( const glm::vec3& position, float yaw, float pitch, float width, float height ) :
        m_position( position ), m_yaw( yaw ), m_pitch( pitch ) {
        set_aspect_ratio( width, height );
        _update_vectors( );
        _update_matrices( );
    }

    void Camera::set_aspect_ratio( float width, float height ) {
        m_aspect_ratio = width / height;

        m_dirtyMatrices = true;
    }

    void Camera::rotate( float deltaYaw, float deltaPitch, float deltaRoll ) {
        m_yaw += deltaYaw;
        m_yaw = std::fmod( m_yaw, 360.0f );

        m_roll += deltaRoll;

        m_pitch -= deltaPitch;
        m_pitch = std::clamp( m_pitch, m_min_pitch, m_max_pitch );

        m_dirtyMatrices = true;

        _update_vectors( );
    }

    const glm::vec3& Camera::get_front( ) const { return m_front; }

    const glm::vec3& Camera::get_right( ) const { return m_right; }

    const glm::vec3& Camera::get_position( ) const { return m_position; }

    void Camera::set_position( const glm::vec3& new_position ) {
        m_position      = new_position;
        m_dirtyMatrices = true;
    }

    const glm::mat4& Camera::get_view_matrix( ) {
        if ( m_dirtyMatrices ) {
            _update_matrices( );
        }

        return m_view_matrix;
    }

    const glm::mat4& Camera::get_projection_matrix( ) {
        if ( m_dirtyMatrices ) {
            _update_matrices( );
        }

        return m_projection_matrix;
    }

    Frustum Camera::get_frustum( ) {
        if ( m_dirtyMatrices ) {
            _update_matrices( );
        }

        Frustum frustum;
        frustum.planes[0] = m_frustum[0];
        frustum.planes[1] = m_frustum[1];
        frustum.planes[2] = m_frustum[2];
        frustum.planes[3] = m_frustum[3];
        frustum.planes[4] = m_frustum[4];
        frustum.planes[5] = m_frustum[5];

        return frustum;
    }

    void Camera::_update_vectors( ) {
        m_front.x = cos( glm::radians( m_yaw ) ) * cos( glm::radians( m_pitch ) );
        m_front.y = sin( glm::radians( m_pitch ) );
        m_front.z = sin( glm::radians( m_yaw ) ) * cos( glm::radians( m_pitch ) );
        m_front   = normalize( m_front );

        m_right = normalize( cross( m_front, m_world_up ) );
        m_up    = normalize( cross( m_right, m_front ) );

        if ( m_roll != 0.0f ) {
            const glm::mat4 roll_matrix = glm::rotate( glm::mat4( 1.0f ), glm::radians( m_roll ), m_front );
            m_right                     = glm::vec3( roll_matrix * glm::vec4( m_right, 0.0f ) );
            m_up                        = glm::vec3( roll_matrix * glm::vec4( m_up, 0.0f ) );
        }

        m_dirtyMatrices = true;
    }

    void Camera::_update_matrices( ) {
        m_view_matrix       = lookAt( m_position, m_position + m_front, m_up );

        // far and near plane are swapped for reverse Z
        m_projection_matrix = glm::perspectiveRH( glm::radians( m_fov ), m_aspect_ratio, m_far_plane, m_near_plane );

        _update_frustum( );

        m_dirtyMatrices = false;
    }

    void Camera::_update_frustum( ) {
        glm::mat4 matrix = m_projection_matrix * m_view_matrix;

        enum side { LEFT   = 0,
                    RIGHT  = 1,
                    TOP    = 2,
                    BOTTOM = 3,
                    BACK   = 4,
                    FRONT  = 5 };

        m_frustum[LEFT].x = matrix[0].w + matrix[0].x;
        m_frustum[LEFT].y = matrix[1].w + matrix[1].x;
        m_frustum[LEFT].z = matrix[2].w + matrix[2].x;
        m_frustum[LEFT].w = matrix[3].w + matrix[3].x;

        m_frustum[RIGHT].x = matrix[0].w - matrix[0].x;
        m_frustum[RIGHT].y = matrix[1].w - matrix[1].x;
        m_frustum[RIGHT].z = matrix[2].w - matrix[2].x;
        m_frustum[RIGHT].w = matrix[3].w - matrix[3].x;

        m_frustum[TOP].x = matrix[0].w - matrix[0].y;
        m_frustum[TOP].y = matrix[1].w - matrix[1].y;
        m_frustum[TOP].z = matrix[2].w - matrix[2].y;
        m_frustum[TOP].w = matrix[3].w - matrix[3].y;

        m_frustum[BOTTOM].x = matrix[0].w + matrix[0].y;
        m_frustum[BOTTOM].y = matrix[1].w + matrix[1].y;
        m_frustum[BOTTOM].z = matrix[2].w + matrix[2].y;
        m_frustum[BOTTOM].w = matrix[3].w + matrix[3].y;

        m_frustum[BACK].x = matrix[0].w + matrix[0].z;
        m_frustum[BACK].y = matrix[1].w + matrix[1].z;
        m_frustum[BACK].z = matrix[2].w + matrix[2].z;
        m_frustum[BACK].w = matrix[3].w + matrix[3].z;

        m_frustum[FRONT].x = matrix[0].w - matrix[0].z;
        m_frustum[FRONT].y = matrix[1].w - matrix[1].z;
        m_frustum[FRONT].z = matrix[2].w - matrix[2].z;
        m_frustum[FRONT].w = matrix[3].w - matrix[3].z;

        for ( auto i = 0; i < 6; i++ ) {
            float length = sqrtf( m_frustum[i].x * m_frustum[i].x + m_frustum[i].y * m_frustum[i].y + m_frustum[i].z * m_frustum[i].z );
            m_frustum[i] /= length;
        }
    }

} // namespace tl