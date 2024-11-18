#version 460

#extension GL_EXT_scalar_block_layout : require

struct Vertex {
    float vx, vy, vz;
};

layout( push_constant, scalar ) uniform PushConstant {
    mat4 view;
    mat4 projection;
}
pc;

void main( ) {
    const Vertex vertices[3] = {
            { 0.0f, -0.5f, 0.0f },
            { 0.5f, 0.5f, 0.0f },
            { -0.5f, 0.5f, 0.0f },
    };

    Vertex vertex   = vertices[gl_VertexIndex];
    vec4   position = pc.projection * pc.view * vec4( vertex.vx, vertex.vy, vertex.vz, 1.0f );

    gl_Position = position;
}