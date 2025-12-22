#version 430 core

#define SH_C0 0.28209479177387814f
#define SH_C1 0.4886025119029199f

#define SH_C2_0 1.0925484305920792f
#define SH_C2_1 -1.0925484305920792f
#define SH_C2_2 0.31539156525252005f
#define SH_C2_3 -1.0925484305920792f
#define SH_C2_4 0.5462742152960396f

#define SH_C3_0 -0.5900435899266435f
#define SH_C3_1 2.890611442640554f
#define SH_C3_2 -0.4570457994644658f
#define SH_C3_3 0.3731763325901154f
#define SH_C3_4 -0.4570457994644658f
#define SH_C3_5 1.445305721320277f
#define SH_C3_6 -0.5900435899266435f

layout(location = 0) in vec2 position;
layout (std430, binding=0) buffer _index {
    int index[];
};
layout (std430, binding=1) buffer _positions {
    float positions[];
};

uniform mat4 projmat;
uniform mat4 viewmat;

out int splat_index;
out vec4 g_pos;
out vec4 g_pos_view;
out vec4 g_pos_screen;

vec3 get_position(int offset)
{
    offset *= 3;
    return vec3(positions[offset + 0 ], positions[offset + 1], positions[offset + 2]);
}

void main()
{
    int splat_idx = index[gl_InstanceID];

    g_pos = vec4(get_position(splat_idx), 1.f);
    g_pos_view = viewmat * g_pos;
    g_pos_screen = projmat * g_pos_view;
    g_pos_screen.xyz = g_pos_screen.xyz / g_pos_screen.w;
    g_pos_screen.w = 1.f;

    if (any(greaterThan(abs(g_pos_screen.xy), vec2(2.0))))
      return;

    if(g_pos_screen.z < 0.5)
      return;

    if(g_pos_screen.z > 1.0)
      return;

    splat_index = splat_idx;
    gl_Position = g_pos_screen;
}
