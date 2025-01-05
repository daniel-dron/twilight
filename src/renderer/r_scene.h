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

#include <array>
#include <optional>
#include <types.h>
#include <vulkan/vulkan_core.h>
#include "r_resources.h"
#include "renderer/r_shaders.h"

namespace tl {

    struct Vertex {
        f32 vx, vy, vz;
        f32 nx, ny, nz;
    };

    const size_t max_vertices  = 64;
    const size_t max_triangles = 124;
    struct Meshlet {
        u64 data_offset; // SceneGeometry::meshlet_data -> [data_offset ... data_offset + vertex_count[ = vertex indices (use index + mesh.vertex_offset to index into vertices_buffer).
                         // After that until index_count, indices
                         //
                         // u32 vindex = meshlet_data[data_offset + i] + mesh.vertex_offset;
                         // Vertex v = vertices[index];

        u32 vertex_count;
        u32 triangle_count; // triangle count (to get index count do x3)

        // backface culling
        f32 cone_apex[3];
        f32 cone_axis[3];
        f32 cone_cutoff;
    };

    struct Lod {
        // Used during mesh pipeline (mesh shaders)
        u32 meshlet_index; // first meshlet index into SceneGeometry::meshlets;
        u32 meshlet_count;

        // Used during vertex pipeline
        u32 index_count;
        u32 index_offset;
    };

    struct SubMesh {
        glm::vec3 center;
        f32       radius;

        glm::vec4 min;
        glm::vec4 max;

        u32 vertex_offset;
        u32 pad;
        u32 pad2;
        u32 pad3;

        Lod lods[6];
    };

#define HAS_FLAG( flags, flag ) ( ( flags & flag ) == flag )

    constexpr uint8_t R_INVALID            = 0;
    constexpr uint8_t R_VALID              = 1 << 0;
    constexpr uint8_t R_VISIBLE_LAST_FRAME = 1 << 1;

    struct SubMeshRenderable {
        glm::mat4 transform;
        u64       mesh;
        u8        flags;
    };

    struct SubMeshRenderableUpdate {
        Handle<SubMeshRenderable> handle;
        SubMeshRenderable         new_value;
    };

    struct SceneGeometry {
        std::vector<Vertex>  vertices;
        std::vector<u32>     indices;
        std::vector<Meshlet> meshlets;
        std::vector<u32>     meshlet_data; // Contains the meshlet data (vertex indices & triangles)
        std::vector<SubMesh> meshes;

        Buffer vertices_buffer;
        Buffer indices_buffer;
        Buffer meshlets_buffer;
        Buffer meshlet_data_buffer;
        Buffer meshes_buffer;
    };

    // This class represents the Scene data that lives on the GPU.
    // Objects that are to be rendered (such as meshes, particles, debug geometry, etc) must be registered on the GPUScene
    class GPUScene {
    public:
        void initialize( );
        void shutdown( );

        Handle<SubMeshRenderable> register_renderable( SubMeshRenderable* renderable );
        const SubMeshRenderable&  get_object( Handle<SubMeshRenderable> handle ) const;
        void                      update_transform( Handle<SubMeshRenderable> handle, const glm::mat4& transform );

        void sync( );

        VkDeviceAddress get_renderables_address( ) const { return m_renderables_gpu.device_address; }

    private:
        static const u64 m_renderables_max = 100'000;

        std::optional<Handle<SubMeshRenderable>> get_free_handle( );                                       // Get a free handle from available spots
        void                                     register_free_handle( Handle<SubMeshRenderable> handle ); // Register a handle as available for take

        Buffer m_renderables_gpu       = { };
        Buffer m_dirty_renderables_gpu = { };

        std::vector<SubMeshRenderable>         m_renderables;
        std::vector<Handle<SubMeshRenderable>> m_free_spots;        // Free spots in m_renderables
        std::vector<SubMeshRenderableUpdate>   m_dirty_renderables; // Indices into m_renderables that need to be updated on the gpu

        struct UpdateRenderablesPushConstants {
            VkDeviceAddress renderables;
            VkDeviceAddress updates;
            u64             count;
        };
        Pipeline m_copy_pipeline = { };
    };

} // namespace tl