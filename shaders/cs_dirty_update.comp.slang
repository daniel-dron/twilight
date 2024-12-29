#include "scene.slang"

struct PushConstants {
    SubMeshRenderable* draws;
    SubMeshRenderableUpdate* updates;
    uint64_t count;
};

[[vk_push_constant]]
PushConstants push_data;

[shader("compute")]
[numthreads(32, 1, 1)]
void main(
    uint3 group_id : SV_GroupID,
    uint3 thread_id : SV_GroupThreadID 
) {
    uint update_id = group_id.x * 32 + thread_id.x;

    if (update_id < push_data.count) {
        SubMeshRenderableUpdate update = push_data.updates[update_id];
        push_data.draws[update.handle.handle] = update.new_value;
    }
}