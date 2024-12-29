#include "common.slang"

struct TaskPayload {
    uint32_t draw_id;
}

struct PushData {
    float4x4 view;
    float4x4 projection;
    float4 camera_position;
    SubMeshRenderable* draws;
    Mesh* meshes;
    Meshlet* meshlets;
    uint32_t* meshlet_data;
    Vertex* vertex_buffer;
    DrawMeshTaskCommand* cmds;
}

[[vk_push_constant]]
PushData push_data;

groupshared TaskPayload payload;

uint gl_DrawID() {
    return spirv_asm {
        OpCapability DrawParameters;
        result:$$uint = OpLoad builtin(DrawIndex:uint);
    };
}

[shader("amplification")]
[numthreads(1, 1, 1)]
void main(
    uint3 group_id : SV_GroupID,
) {
    uint32_t meshlet_id = group_id.x;
    
    DrawMeshTaskCommand cmd = push_data.cmds[gl_DrawID()];
    
    SubMeshRenderable draw = push_data.draws[cmd.draw_id];
    payload.draw_id = cmd.draw_id;
    
    DispatchMesh(1, 1, 1, payload);
}