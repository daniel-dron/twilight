#include "common.slang"

struct VertexOutput {
    float4 position : SV_Position;
    float3 color: COLOR0;
}

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
}

[[vk_push_constant]]
PushData push_data;

[shader("mesh")]
[numthreads(1, 1, 1)]
[outputtopology("line")]
void main(
   in payload TaskPayload payload,
   out indices uint2 primitives[12],
   out vertices VertexOutput vertices[8]
) {
   SubMeshRenderable draw = push_data.draws[payload.draw_id];
   Mesh mesh = push_data.meshes[draw.mesh];

   SetMeshOutputCounts(8, 12);

   float3 corners[8] = {
       float3(mesh.min.x, mesh.min.y, mesh.min.z),
       float3(mesh.max.x, mesh.min.y, mesh.min.z),
       float3(mesh.min.x, mesh.max.y, mesh.min.z),
       float3(mesh.max.x, mesh.max.y, mesh.min.z),
       float3(mesh.min.x, mesh.min.y, mesh.max.z),
       float3(mesh.max.x, mesh.min.y, mesh.max.z), 
       float3(mesh.min.x, mesh.max.y, mesh.max.z),
       float3(mesh.max.x, mesh.max.y, mesh.max.z)
   };

   for(uint i = 0; i < 8; i++) {
       float4 worldPos = mul(draw.model, float4(corners[i], 1.0));
       vertices[i].position = mul(push_data.projection, mul(push_data.view, worldPos));
       vertices[i].color = float3(1.0, 0.0, 0.0);
   }

   primitives[0] = uint2(0,1);
   primitives[1] = uint2(1,3);
   primitives[2] = uint2(3,2);
   primitives[3] = uint2(2,0);
   primitives[4] = uint2(4,5);
   primitives[5] = uint2(5,7);
   primitives[6] = uint2(7,6);
   primitives[7] = uint2(6,4);
   primitives[8] = uint2(0,4);
   primitives[9] = uint2(1,5);
   primitives[10] = uint2(2,6);
   primitives[11] = uint2(3,7);
}