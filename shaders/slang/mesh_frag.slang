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
    float3 color = coarseVertex.color;

    Fragment fragment;
    fragment.color = float4(color, 1.0f);

    return fragment;
}