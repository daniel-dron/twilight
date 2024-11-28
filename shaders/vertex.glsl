struct Vertex {
    float vx, vy, vz;
    float nx, ny, nz;
};

layout( buffer_reference, scalar, buffer_reference_align = 8 ) readonly buffer VertexBuffer {
    Vertex vertices[];
};