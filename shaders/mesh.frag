#version 460

layout( location = 0 ) out vec4 out_color;

layout( location = 0 ) in flat uint primitive_index;

uint murmurHash( uint idx ) {
    uint m = 0x5bd1e995;
    uint r = 24;

    uint h = 64684;
    uint k = idx;

    k *= m;
    k ^= ( k >> r );
    k *= m;
    h *= m;
    h ^= k;

    return h;
}

void main( ) {
    uint colorPacked = murmurHash( primitive_index );
    out_color        = unpackUnorm4x8( colorPacked );
}