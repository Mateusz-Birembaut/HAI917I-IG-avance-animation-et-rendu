// NOTE: Shader automatically converted from Godot Engine 4.5.1.stable's StandardMaterial3D.

shader_type spatial;
render_mode blend_mix, depth_draw_opaque, cull_back, diffuse_burley, specular_schlick_ggx;

uniform vec4 albedo : source_color;
uniform sampler2D texture_albedo : source_color, filter_linear_mipmap, repeat_enable;
uniform ivec2 albedo_texture_size;
uniform float point_size : hint_range(0.1, 128.0, 0.1);

uniform float roughness : hint_range(0.0, 1.0);
uniform sampler2D texture_metallic : hint_default_white, filter_linear_mipmap, repeat_enable;
uniform vec4 metallic_texture_channel;
uniform sampler2D texture_roughness : hint_roughness_r, filter_linear_mipmap, repeat_enable;

uniform sampler2D texture_colormap : source_color, filter_linear, repeat_disable;
uniform sampler2D texture_noise : source_color, filter_linear, repeat_disable;

uniform float specular : hint_range(0.0, 1.0, 0.01);
uniform float metallic : hint_range(0.0, 1.0, 0.01);

uniform float min_height : hint_range(-50.0, 50.0, 0.5);
uniform float max_height : hint_range(-50.0, 50.0, 0.5);

uniform vec3 uv1_scale;
uniform vec3 uv1_offset;
uniform vec3 uv2_scale;
uniform vec3 uv2_offset;

#include "noise.gdshaderinc"

varying vec3 world_pos;

void vertex() {
	UV = UV * uv1_scale.xy + uv1_offset.xy;
	//VERTEX

	float H = 1.1f;
	float offset = 0.01f;

	vec3 p = VERTEX;
	p.y = fbm(p.xz, H);

	vec3 p2 = VERTEX;
	p2.x += offset;
	p2.y = fbm(p2.xz, H);

	vec3 p3 = VERTEX;
	p3.z += offset;
	p3.y = fbm(p3.xz, H);

	VERTEX = p;

	//NORMAL
	NORMAL = normalize(cross(p3-p, p2-p)) ;

	world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	vec2 base_uv = UV;

	vec2 sample_colormap = vec2((world_pos.y - min_height) / (max_height - min_height) , 0);
	vec4 color = texture(texture_colormap, sample_colormap);

	vec4 albedo_tex = texture(texture_albedo, base_uv);
	ALBEDO = albedo.rgb * albedo_tex.rgb * color.rgb;

	float metallic_tex = dot(texture(texture_metallic, base_uv), metallic_texture_channel);
	METALLIC = metallic_tex * metallic;
	SPECULAR = specular;

	vec4 roughness_texture_channel = vec4(1.0, 0.0, 0.0, 0.0);
	float roughness_tex = dot(texture(texture_roughness, base_uv), roughness_texture_channel);
	ROUGHNESS = roughness_tex * roughness;

}
