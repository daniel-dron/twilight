struct SubMeshRenderable {
    float4x4 model;
    uint64_t mesh;
    uint8_t flags;
}

struct DrawMeshTaskCommand {
    uint32_t groupCountX;
    uint32_t groupCountY;
    uint32_t groupCountZ;
    uint32_t draw_id;
    uint32_t lod_id;
    float pad;
};