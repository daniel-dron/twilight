#include "draws.slang"

struct Vertex {
    float vx, vy, vz;
    float nx, ny, nz;
}

struct Meshlet {
    uint64_t data_offset;
    uint32_t vertex_count;
    uint32_t triangle_count;

    float cone_apex[3];
    float cone_axis[3];
    float cone_cutoff;
}

struct Lod {
    uint32_t meshlet_index;
    uint32_t meshlet_count;
}

struct Mesh {
    float3 center;
    float radius;

    float4 min;
    float4 max;
    
    uint32_t vertex_offset;
    uint32_t pad;
    uint32_t pad2;
    uint32_t pad3;

    Lod lods[6];
}

struct CullData {
    SubMeshRenderable* draws;
    DrawMeshTaskCommand* cmds;
    Mesh* meshes;
    uint32_t* visibility;
    float4x4 projection_matrix;
    float4x4 view_matrix;
    uint64_t count;
    uint32_t* out_count;
    float3 camera_position;
    uint32_t enable_lod;
    float4 frustum[6];
    float4 occlusion_data; // width, height, znear, enabled
    float4 occlusion_data2; // p00, p11
};

struct DrawCommandCullData {
    CullData* cull_data;
    float64_t time;
};