#version 330

uniform mat4 projection_matrix;

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

in VertexData
{
	vec3 view;
	vec2 texCoord;
}
inData[];

out GeoData
{
    vec3 normal;
	vec3 view;
	vec2 texCoord;
}
outData;

void main()
{
    vec3 x0 = gl_in[0].gl_Position.xyz;
    vec3 x1 = gl_in[1].gl_Position.xyz;
    vec3 x2 = gl_in[2].gl_Position.xyz;

    vec3 eye_n = normalize(cross(x1 - x0, x2 - x0));

    vec4 p0 = projection_matrix * gl_in[0].gl_Position;
    vec4 p1 = projection_matrix * gl_in[1].gl_Position;
    vec4 p2 = projection_matrix * gl_in[2].gl_Position;

    gl_Position = p0;
	outData.normal = eye_n;
	outData.view = inData[0].view;
	outData.texCoord = inData[0].texCoord;
    EmitVertex();

    gl_Position = p1;
	outData.normal = eye_n;
	outData.view = inData[1].view;
	outData.texCoord = inData[1].texCoord;
    EmitVertex();

    gl_Position = p2;
	outData.normal = eye_n;
	outData.view = inData[2].view;
	outData.texCoord = inData[2].texCoord;
    EmitVertex();
}
