[[vk_binding(0)]] Texture2D<float4> accumulation_texture;
[[vk_binding(1)]] RWTexture2D<float4> visualization_texture;

[shader("compute")]
[numthreads(32, 32, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID) {
    uint2 coord = dispatch_id.xy;
    float4 accumulated = accumulation_texture.Load(int3(coord, 0));

    float overdraw_count = accumulated.r / 0.1f;

    float4 color = {};
    if (overdraw_count >= 5.0)
    {
        color = float4(1.0, 0.0, 0.0, 1.0);  // Bright red for heavy overdraw
    }
    else if (overdraw_count >= 3.0)
    {
        color = float4(1.0, 0.5, 0.0, 1.0);  // Orange for medium overdraw
    }
    else if (overdraw_count >= 1.0)
    {
        color = float4(0.0, 1.0, 0.0, 1.0);  // Green for light overdraw
    }
    else
    {
        color = float4(0.0, 0.0, 0.2, 1.0);  // Dark blue for no overdraw
    }

    visualization_texture[coord] = color;
}