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

        m_pitch += deltaPitch;
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
        m_projection_matrix = glm::perspective( glm::radians( m_fov ), m_aspect_ratio, m_near_plane, m_far_plane );
        m_projection_matrix[1][1] *= -1;

        m_dirtyMatrices = false;
    }

} // namespace tl