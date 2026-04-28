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

uniform mat4 projmat;
uniform mat4 viewmat;
uniform vec3 cam_pos;
uniform vec2 tanxy;
uniform float focal;
uniform int max_sh_dim;
uniform int render_mod;

// Input : suppose que le VS envoie un point par instance (ou autre)
layout(points) in;

layout (std430, binding=0) buffer _index {
    int index[];
};
layout (std430, binding=1) buffer _positions {
    float positions[];
};
layout (std430, binding=2) buffer _rotations {
    float rotations[];
};
layout (std430, binding=3) buffer _scales {
    float scales[];
};
layout (std430, binding=4) buffer _opacities {
    float opacities[];
};
layout (std430, binding=5) buffer _sh {
    float sh[];
};

// Output : QUAD via triangle strip
layout(triangle_strip, max_vertices = 4) out;

in int splat_index[];
in vec4 g_pos[];
in vec4 g_pos_view[];
in vec4 g_pos_screen[];

// Sortie finale vers le fragment shader
out vec3 color;
out float alpha;
out vec3 conic;
out vec2 coordxy;  // local coordinate in quad, unit in pixel
out float discardThresold;

mat3 computeCov3D(vec3 scale, vec4 q)  // should be correct
{
    mat3 S = mat3(0.f);
    S[0][0] = scale.x;
    S[1][1] = scale.y;
    S[2][2] = scale.z;
    float r = q.x;
    float x = q.y;
    float y = q.z;
    float z = q.w;

    mat3 R = mat3(
        1.f - 2.f * (y * y + z * z), 2.f * (x * y - r * z), 2.f * (x * z + r * y),
        2.f * (x * y + r * z), 1.f - 2.f * (x * x + z * z), 2.f * (y * z - r * x),
        2.f * (x * z - r * y), 2.f * (y * z + r * x), 1.f - 2.f * (x * x + y * y)
    );

    mat3 M = S * R;
    mat3 Sigma = transpose(M) * M;
    return Sigma;
}

vec3 computeCov2D(vec4 mean_view, float focal_x, float focal_y, float tan_fovx, float tan_fovy, mat3 cov3D, mat4 viewmatrix)
{
    vec4 t = mean_view;
    // why need this? Try remove this later
    float limx = 1.3f * tan_fovx;
    float limy = 1.3f * tan_fovy;
    float txtz = t.x / t.z;
    float tytz = t.y / t.z;
    t.x = min(limx, max(-limx, txtz)) * t.z;
    t.y = min(limy, max(-limy, tytz)) * t.z;

    mat3 J = mat3(
        focal_x / t.z, 0.0f, -(focal_x * t.x) / (t.z * t.z),
        0.0f, focal_y / t.z, -(focal_y * t.y) / (t.z * t.z),
        0, 0, 0
    );
    mat3 W = transpose(mat3(viewmatrix));
    mat3 T = W * J;

    mat3 cov = transpose(T) * transpose(cov3D) * T;
    // Apply low-pass filter: every Gaussian should be at least
    // one pixel wide/high. Discard 3rd row and column.
    cov[0][0] += 0.3f;
    cov[1][1] += 0.3f;
    return vec3(cov[0][0], cov[0][1], cov[1][1]);
}

vec3 get_position(int offset)
{
    offset *= 3;
    return vec3(positions[offset + 0 ], positions[offset + 1], positions[offset + 2]);
}

vec4 get_rotation(int offset)
{
    offset *= 4;
    return vec4(rotations[offset + 0], rotations[offset + 1], rotations[offset + 2], rotations[offset + 3]);
}

vec3 get_scale(int offset)
{
  offset *= 3;
  return vec3(scales[offset + 0], scales[offset + 1], scales[offset + 2]);
}

float get_opacity(int offset)
{
  return opacities[offset];
}

vec3 get_sh(int offset, int entry)
{
  offset = offset * max_sh_dim + entry;
  return vec3(sh[offset + 0], sh[offset + 1], sh[offset + 2]) ;
}

void main()
{
  int splat_idx = splat_index[0];

  vec4 g_rot = get_rotation(splat_idx);
  vec3 g_scale = get_scale(splat_idx);
  float g_opacity = get_opacity(splat_idx);

  mat3 cov3d = computeCov3D(g_scale, g_rot);
  vec2 wh = 2 * tanxy * focal;
  vec3 cov2d = computeCov2D(g_pos_view[0], focal, focal, tanxy.x, tanxy.y, cov3d, viewmat);

  // Invert covariance (EWA algorithm)
  float det = (cov2d.x * cov2d.z - cov2d.y * cov2d.y);
  if (det == 0.0f)
    return;

  float det_inv = 1.f / det;
  conic = vec3(cov2d.z * det_inv, -cov2d.y * det_inv, cov2d.x * det_inv);

  alpha = g_opacity;
  discardThresold = -log(255.0 * alpha);

  // Convert SH to color
  int sh_start = splat_idx;
  vec3 dir = g_pos[0].xyz - cam_pos;
  dir = normalize(dir);

  color = SH_C0 * get_sh(sh_start, 0);
  color += 0.5f;

  float s = 1.0;
  vec2 dirs[4] = vec2[4](
        vec2(-s,  s),
        vec2( s,  s),
        vec2(-s, -s),
        vec2( s, -s)
  );

  vec2 quadwh_scr = vec2(3.f * sqrt(cov2d.x), 3.f * sqrt(cov2d.z));  // screen space half quad height and width
  vec2 quadwh_ndc = quadwh_scr / wh * 2;  // in ndc space

  for (int i = 0; i < 4; i++)
  {
    gl_Position = vec4(g_pos_screen[0].xy + dirs[i] * quadwh_ndc, g_pos_screen[0].z, g_pos_screen[0].w);
    coordxy = dirs[i]*quadwh_scr;
    EmitVertex();
  }

  EndPrimitive();
}
