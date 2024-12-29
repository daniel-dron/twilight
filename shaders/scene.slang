#include "draws.slang"

struct Handle {
    uint32_t handle;
};

struct SubMeshRenderableUpdate {
    Handle handle;
    SubMeshRenderable new_value;
};