[[vk_binding(0)]] Texture2D<float> depth;
[[vk_binding(1)]] RWTexture2D<float4> out_color;

const static float g_near = 0.1f;
const static float g_far = 1000.0f;

float linearize_depth(float depth)
{
    float z = depth * 2.0 - 1.0;
    return (2.0 * g_near * g_far) / (g_far + g_near - z * (g_far - g_near));
}

float remap_for_display(float depth)
{
    // Remap from [near,far] to [0,1]
    float remapped = (depth - g_near) / (g_far - g_near);
    
    // gamma curve to make the middle values more visible
    return pow(remapped, 0.4545);
}

[shader("compute")]
[numthreads(32, 32, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint2 coord = dispatch_id.xy;
    float d = depth[coord];
    
    float linear_depth = linearize_depth(d);
    float display_depth = remap_for_display(linear_depth);
    
    out_color[coord] = float4(display_depth.xxx, 1.0f);
}