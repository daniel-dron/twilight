#extension GL_EXT_shader_8bit_storage : require
#extension GL_ARB_gpu_shader_int64 : enable

#include "vertex.glsl"

struct Meshlet {
    uint vertex_offset;
    uint triangle_offset;
    uint vertex_count;
    uint triangle_count;

    float cone_apex[3];
    float cone_axis[3];
    float cone_cutoff;
};

layout( buffer_reference, scalar, buffer_reference_align = 8 ) readonly buffer VertexBuffer {
    Vertex vertices[];
};

layout( buffer_reference, scalar, buffer_reference_align = 8 ) readonly buffer MeshletBuffer {
    Meshlet meshlets[];
};

layout( buffer_reference, scalar, buffer_reference_align = 8 ) readonly buffer MeshletVertices {
    uint vertices[];
};

layout( buffer_reference, scalar, buffer_reference_align = 8 ) readonly buffer MeshletTriangles {
    uint8_t triangles[];
};

struct Mesh {
    VertexBuffer     vertex_buffer;
    MeshletBuffer    meshlet_buffer;
    MeshletVertices  meshlet_vertices;
    MeshletTriangles meshlet_triangles;
    uint64_t         meshlet_count;
};

layout( buffer_reference, scalar, buffer_reference_align = 8 ) readonly buffer MeshesBuffer {
    Mesh meshes[];
};