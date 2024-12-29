#include "common.slang"

struct TaskPayload {
    uint32_t meshlet_indices[32];
    uint32_t draw_id;
    uint32_t lod_id;
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

bool cone_cull(float3 view_direction, float3 cone_axis, float cone_cutoff, float4x4 model) {
    float4 transformed_cone_axis = mul(model, float4(cone_axis, 0.0f));
    transformed_cone_axis = normalize(transformed_cone_axis);

    float dot_product = dot(view_direction, transformed_cone_axis.xyz);
    return dot_product >= cone_cutoff;
}

groupshared uint32_t meshlet_count;
groupshared TaskPayload payload;

uint gl_DrawID() {
    return spirv_asm {
        OpCapability DrawParameters;
        result:$$uint = OpLoad builtin(DrawIndex:uint);
    };
}

[shader("amplification")]
[numthreads(32, 1, 1)]
void main(
    uint3 group_id : SV_GroupID,
    uint3 invocation_id : SV_GroupThreadID
) {
    uint32_t meshlet_id = group_id.x * 32 + invocation_id.x;
    
    DrawMeshTaskCommand cmd = push_data.cmds[gl_DrawID()];
    
    SubMeshRenderable draw = push_data.draws[cmd.draw_id];
    payload.draw_id = cmd.draw_id;
    payload.lod_id = cmd.lod_id;
    
    Mesh mesh = push_data.meshes[draw.mesh];
    Lod lod = mesh.lods[cmd.lod_id];
    
    meshlet_count = 0;
    GroupMemoryBarrierWithGroupSync();
    
    if (meshlet_id < lod.meshlet_count) {
        Meshlet meshlet = push_data.meshlets[lod.meshlet_index + meshlet_id];
        
        float4 apex = mul(draw.model, float4(meshlet.cone_apex[0], meshlet.cone_apex[1], meshlet.cone_apex[2], 1.0f));
        float3 view_direction = normalize(push_data.camera_position.xyz - apex.xyz);
        float4 cone_axis = float4(meshlet.cone_axis[0], meshlet.cone_axis[1], meshlet.cone_axis[2], 0.0f);
        
        if (!cone_cull(view_direction, cone_axis.xyz, meshlet.cone_cutoff, draw.model)) {
            uint32_t idx;
            InterlockedAdd(meshlet_count, 1, idx);
            payload.meshlet_indices[idx] = meshlet_id;
        }
    }
    
    GroupMemoryBarrierWithGroupSync();
    DispatchMesh(meshlet_count, 1, 1, payload);
}