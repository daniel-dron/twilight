// https://web.archive.org/web/20200207113336/http://lolengine.net/blog/2013/07/27/rgb-to-hsv-in-glsl
float3 hsv_to_rgb(float3 hsv) {
    float4 K = float4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    float3 p = abs(fract(hsv.xxx + K.xyz) * 6.0 - K.www);
    return hsv.z * lerp(K.xxx, saturate(p - K.xxx), hsv.y);
}

float3 random_color_from_u32(uint32_t draw_id) {
    float hue = (draw_id % 360) / 360.0f;
    return hsv_to_rgb(float3(hue, 1.0, 1.0));
}