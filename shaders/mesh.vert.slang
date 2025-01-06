#include "common.slang"
#include "utils.slang"

struct VertexOutput {
    float4 position : SV_Position;
    float3 color: COLOR0;
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
    DrawMeshTaskCommand* tasks;
    VkDrawIndexedIndirectCommand* cmds;
}

[[vk_push_constant]]
PushData push_data;

uint gl_DrawID() {
    return spirv_asm {
        OpCapability DrawParameters;
        result:$$uint = OpLoad builtin(DrawIndex:uint);
    };
}

[shader("vertex")]
VertexOutput main( uint vertex_id : SV_VertexID ) {
    VkDrawIndexedIndirectCommand command = push_data.cmds[gl_DrawID()];
    SubMeshRenderable draw = push_data.draws[command.draw_id];

    Vertex vertex = push_data.vertex_buffer[vertex_id];

    VertexOutput vertex_out;
    vertex_out.color = random_color_from_u32(command.draw_id);  //float3(vertex.nx, vertex.ny, vertex.nz) * 0.5f + 0.5f;
    vertex_out.position = mul(push_data.projection, mul(push_data.view, mul(draw.model, float4(vertex.vx, vertex.vy, vertex.vz, 1.0f))));

    return vertex_out;
}