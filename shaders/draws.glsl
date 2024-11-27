#extension GL_ARB_gpu_shader_int64 : enable

struct Draw {
    mat4     model;
    uint64_t mesh;
    uint64_t pad;
};

layout( buffer_reference, scalar, buffer_reference_align = 8 ) readonly buffer DrawsBuffer {
    Draw draws[];
};