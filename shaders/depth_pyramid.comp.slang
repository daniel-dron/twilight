[[vk_binding(0)]] Texture2D<float> depth;
[[vk_binding(1)]] Texture2D<float> depth_mips[];
[[vk_binding(2)]] RWTexture2D<float> out_color[];
[[vk_binding(3)]] SamplerState sampler;

[[vk_push_constant]]
uint3 push_data;

const static float g_near = 0.1f;
const static float g_far = 500.0f;

float linearize_depth(float depth)
{
    float z = depth * 2.0 - 1.0;
    return (2.0 * g_near * g_far) / (g_far + g_near - z * (g_far - g_near));
}

float remap_for_display(float depth)
{
    float remapped = (depth - g_near) / (g_far - g_near);
    return pow(remapped, 0.4545);
}

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
        d = depth_mips[push_data.z - 1].Sample(sampler, uv).x;
    }
    
    out_color[push_data.z][coord] = d;
}