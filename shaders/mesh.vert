#version 460

#extension GL_EXT_scalar_block_layout : require
#extension GL_EXT_buffer_reference : require

struct Vertex {
    float vx, vy, vz;
};

layout( buffer_reference, scalar, buffer_reference_align = 8 ) readonly buffer VertexBuffer {
    Vertex vertices[];
};

layout( push_constant, scalar ) uniform PushConstant {
    mat4         view;
    mat4         projection;
    VertexBuffer vertex_buffer;
}
pc;

void main( ) {
    Vertex vertex   = pc.vertex_buffer.vertices[gl_VertexIndex];
    vec4   position = pc.projection * pc.view * vec4( vertex.vx, vertex.vy, vertex.vz, 1.0f );

    gl_Position = position;
}