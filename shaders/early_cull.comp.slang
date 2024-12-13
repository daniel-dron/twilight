#include "common.slang"

[[vk_push_constant]]
DrawCommandCullData push_data;

[shader("compute")]
[numthreads(32, 1, 1)]
void main(
    uint3 group_id : SV_GroupID,
    uint3 thread_id : SV_GroupThreadID 
) {
    
    uint cmd_id = group_id.x * 32 + thread_id.x;
    uint lod_id = 0;

    if ( cmd_id < push_data.cull_data.count) {
        Draw draw = push_data.cull_data.draws[cmd_id];
        Mesh mesh = push_data.cull_data.meshes[draw.mesh];

        float3 center = mul(draw.model, float4(mesh.center, 1.0f)).xyz;
        float radius = mesh.radius * length(draw.model[0].xyz);

        bool visible = true;
        for ( int i = 0; i < 6; i++ ) {
            visible = visible && ( ( dot( push_data.cull_data.frustum[i], float4( center, 1.0f ) ) ) > - radius );
        }
        
        // Draw meshes that *WERE* visible last frame (and that pass the current frustum check)
        if (visible && push_data.cull_data.visibility[cmd_id] == 1) {
            uint id;
            InterlockedAdd(push_data.cull_data.out_count[0], 1, id);

            if (push_data.cull_data.enable_lod == 1) {
                float dist = distance(center, push_data.cull_data.camera_position);
                float size = (radius * 300.0) / max(dist * dist, 0.001); 
                float lod_distance = -log2(clamp(size, 0.01, 1.0));
                lod_id = clamp(int(lod_distance * 0.75), 0, 5);
            }

            Lod lod = mesh.lods[lod_id];

            push_data.cull_data.cmds[id].groupCountX = ( lod.meshlet_count + 31 ) / 32;
            push_data.cull_data.cmds[id].groupCountY = 1;
            push_data.cull_data.cmds[id].groupCountZ = 1;
            push_data.cull_data.cmds[id].draw_id = cmd_id;
            push_data.cull_data.cmds[id].lod_id = lod_id;
        }
    }
}