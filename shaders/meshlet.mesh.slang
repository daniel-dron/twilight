#include "common.slang"

struct VertexOutput {
    float4 position : SV_Position;
    float3 color: COLOR0;
}

struct TaskPayload {
    uint32_t meshlet_indices[32];
    uint32_t draw_id;
    uint32_t lod_id;
}

struct PushData {
    float4x4 view;
    float4x4 projection;
    float4 camera_position;
    Draw* draws;
    Mesh* meshes;
    Meshlet* meshlets;
    uint32_t* meshlet_data;
    Vertex* vertex_buffer;
}

[[vk_push_constant]]
PushData push_data;

float3 get_meshlet_debug_color(uint meshlet_id) {
    static const float goldenConj = 0.618033988749895f;
    float hue = fmod(float(meshlet_id) * goldenConj, 1.0f);
    
    // Convert HSV to RGB with S=0.95, V=0.95
    float h = hue * 6.0f;
    static const float s = 0.95f;
    static const float v = 0.95f;
    
    float c = v * s;
    float x = c * (1.0f - abs(fmod(h, 2.0f) - 1.0f));
    float m = v - c;
    
    float3 rgb;
    if(h < 1.0f)      { rgb = float3(c, x, 0); }
    else if(h < 2.0f) { rgb = float3(x, c, 0); }
    else if(h < 3.0f) { rgb = float3(0, c, x); }
    else if(h < 4.0f) { rgb = float3(0, x, c); }
    else if(h < 5.0f) { rgb = float3(x, 0, c); }
    else              { rgb = float3(c, 0, x); }
    
    return rgb + m;
}

[shader("mesh")]
[numthreads(32, 1, 1)]
[outputtopology("triangle")]
void main(
    in payload TaskPayload payload,
    out indices uint3 triangles[126],
    out vertices VertexOutput vertices[64],
    uint32_t group_id : SV_GroupID,
    uint32_t invocation_id : SV_GroupThreadID
) {
    uint32_t meshlet_id = payload.meshlet_indices[group_id];
    Draw draw = push_data.draws[payload.draw_id];

    Mesh mesh = push_data.meshes[draw.mesh];
    Lod lod = mesh.lods[payload.lod_id];
    Meshlet meshlet = push_data.meshlets[lod.meshlet_index + meshlet_id];

    SetMeshOutputCounts(meshlet.vertex_count, meshlet.triangle_count);

    for (uint32_t i = invocation_id; i < meshlet.vertex_count; i += 32) {
        uint32_t vertex_index = push_data.meshlet_data[meshlet.data_offset + i] + mesh.vertex_offset;
        Vertex vertex = push_data.vertex_buffer[vertex_index];

        VertexOutput vertex_out;
        vertex_out.color = float3(vertex.nx, vertex.ny, vertex.nz) * 0.5f + 0.5f;
        vertex_out.position = mul(push_data.projection, mul(push_data.view, mul(draw.model, float4(vertex.vx, vertex.vy, vertex.vz, 1.0f))));

        vertices[i] = vertex_out;
    }

    for(uint32_t i = invocation_id; i < meshlet.triangle_count; i += 32) {
        uint32_t index = i * 3;
        uint32_t offset = uint32_t(meshlet.data_offset + meshlet.vertex_count);
        triangles[i] = uint3(
            push_data.meshlet_data[index + offset],
            push_data.meshlet_data[index + offset + 1],
            push_data.meshlet_data[index + offset + 2],
        );
    }
}