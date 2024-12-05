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
    uint32_t vertex_offset;
    uint32_t pad;

    Lod lods[6];
}