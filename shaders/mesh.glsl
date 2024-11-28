#extension GL_EXT_shader_8bit_storage : require
#extension GL_ARB_gpu_shader_int64 : enable

#include "vertex.glsl"

struct Meshlet {
    uint64_t data_offset; // SceneGeometry::meshlet_data -> [data_offset ... data_offset + vertex_count[ = vertex indices (use index + mesh.vertex_offset to index into vertices_buffer).
                          // After that until index_count, indices packed
                          //
                          // u32 vindex = meshlet_data[data_offset + i] + mesh.vertex_offset;
                          // Vertex v = vertices[index];
    uint     vertex_count;
    uint     triangle_count; // triangle count (to get index count do x3)

    float cone_apex[3];
    float cone_axis[3];
    float cone_cutoff;
};

layout( buffer_reference, scalar, buffer_reference_align = 8 ) readonly buffer MeshletBuffer {
    Meshlet meshlets[];
};

struct Lod {
    uint meshlet_index;
    uint meshlet_count;
};

struct Mesh {
    vec3  center;
    float radius;
    uint  vertex_offset;
    uint  pad;

    Lod lods[6];
};

layout( buffer_reference, scalar, buffer_reference_align = 8 ) readonly buffer MeshesBuffer {
    Mesh meshes[];
};

layout( buffer_reference, scalar, buffer_reference_align = 8 ) readonly buffer MeshletDataBuffer {
    uint data[];
};