[[vk_binding(0)]] Texture2D<float> depth;
[[vk_binding(1)]] Texture2D<float> depth_mips[];
[[vk_binding(2)]] RWTexture2D<float> out_color[];
[[vk_binding(3)]] SamplerState sampler;

[[vk_push_constant]]
uint3 push_data;

[shader("compute")]
[numthreads(32, 32, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint2 coord = dispatch_id.xy;
    float2 uv = (float2(coord) + 0.5f) / float2(push_data.xy);

    float d;
    if (push_data.z == 0) {
        d = depth.Sample(sampler, uv);
    } else {
        float4 depths = depth_mips[push_data.z - 1].GatherRed(sampler, uv);
        d = max(max(depths.x, depths.y), max(depths.z, depths.w));
    }
    
    out_color[push_data.z][coord] = d;
}