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
    vec4         camera_position;
    VertexBuffer vertex_buffer;
}
pc;

layout( location = 0 ) out uint out_primitive_index;

void main( ) {
    Vertex vertex       = pc.vertex_buffer.vertices[gl_VertexIndex];
    vec4   position     = pc.projection * pc.view * vec4( vertex.vx, vertex.vy, vertex.vz, 1.0f );
    out_primitive_index = int( gl_VertexIndex );
    gl_Position         = position;
}