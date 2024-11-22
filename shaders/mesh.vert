#version 460

#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_scalar_block_layout : require
#extension GL_EXT_buffer_reference : require

#include "vertex.glsl"

layout( buffer_reference, scalar, buffer_reference_align = 8 ) readonly buffer VertexBuffer {
    Vertex vertices[];
};

layout( push_constant, scalar ) uniform PushConstant {
    mat4         view;
    mat4         projection;
    mat4         model;
    vec4         camera_position;
    VertexBuffer vertex_buffer;
}
pc;

layout( location = 0 ) out vec3 color;

void main( ) {
    Vertex vertex   = pc.vertex_buffer.vertices[gl_VertexIndex];
    vec4   position = pc.projection * pc.view * pc.model * vec4( vertex.vx, vertex.vy, vertex.vz, 1.0f );
    color           = vec3( vertex.nx, vertex.ny, vertex.nz ) * 0.5 + 0.5;
    gl_Position     = position;
}