struct CoarseVertex {
    float3 color;
}

struct Fragment {
    float4 color;
}

[shader("fragment")]
Fragment main(
    CoarseVertex coarseVertex : CoarseVertex
) : SV_Target {
    Fragment fragment;
    fragment.color = float4(0.1f, 0.0f, 0.0f, 1.0f);

    return fragment;
}