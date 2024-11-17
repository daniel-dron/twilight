#version 460

struct Vertex {
    float vx, vy, vz;
};

void main( ) {
    const Vertex vertices[3] = {
            { 0.0f, -0.5f, 0.0f },
            { 0.5f, 0.5f, 0.0f },
            { -0.5f, 0.5f, 0.0f },
    };

    Vertex vertex = vertices[gl_VertexIndex];

    gl_Position = vec4( vertex.vx, vertex.vy, vertex.vz, 1.0f );
}