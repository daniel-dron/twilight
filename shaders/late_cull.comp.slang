#include "common.slang"

[[vk_push_constant]]
DrawCommandCullData push_data;

[[vk_binding(0)]] Texture2D<float> depth_pyramid;
[[vk_binding(1)]] SamplerState sampler;

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

        float4 center = mul(draw.model, float4(mesh.center, 1.0f));
        float radius = mesh.radius * length(draw.model[0].xyz);

        bool visible = true;
        for ( int i = 0; i < 6; i++ ) {
            visible = visible && ( ( dot( push_data.cull_data.frustum[i], float4( center.xyz, 1.0f ) ) ) > - radius );
        }

        float color = 0.0f;
        if (visible && push_data.cull_data.occlusion_data.w == 1) {
            float4 center_view = mul(push_data.cull_data.view_matrix, center);

            if (center_view.z < -0.1f) {
                // calculate radius for mip level selection
                float4 center_proj = mul(push_data.cull_data.projection_matrix, float4(center_view.xyz, 1.0));
                float4 radius_proj = mul(push_data.cull_data.projection_matrix, float4(center_view.xyz + float3(radius, 0, 0), 1.0));
                float2 center_screen = center_proj.xy / center_proj.w;
                float2 radius_screen = radius_proj.xy / radius_proj.w;
                float screen_radius = length(radius_screen - center_screen);

                float mip_level = screen_radius > 0.0f ? floor(log2(screen_radius * 1024.0f)) : 0.0f; // TODO: this +1.0f is bad. Doing it to decrease popping but still bad
                mip_level = max(0.0f, mip_level);
                
                // bring the center point closer to the camera so we sample the closest point
                center_view.z += radius;

                float4 clip_center = mul(push_data.cull_data.projection_matrix, center_view);            
                float3 ndc_center = clip_center.xyz / clip_center.w;
                if (all(ndc_center >= 0.0f) || all(ndc_center <= 1.0f)) {
                    float2 uvs = float2(ndc_center.x * 0.5f + 0.5f, ndc_center.y * 0.5f + 0.5f);

                    float projected_depth_center = ndc_center.z;
                    float stored_depth = depth_pyramid.SampleLevel(sampler, uvs, mip_level).x;
                    
                    visible = visible && projected_depth_center < stored_depth;
                }
            }
        }

        // Draw meshes that were *NOT* visible last frame (and that pass the current frustum check)
        if (visible && push_data.cull_data.visibility[cmd_id] == 0) {
            uint id;
            InterlockedAdd(push_data.cull_data.out_count[0], 1, id);

            if (push_data.cull_data.enable_lod == 1) {
                float dist = distance(center.xyz, push_data.cull_data.camera_position);
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
        
        push_data.cull_data.visibility[cmd_id] = visible ? 1 : 0;
    }
}