#version 330

uniform mat4 modelview_matrix;
uniform mat4 projection_matrix;

layout(location = 0) in vec3 position;
layout(location = 1) in vec2 texCoord;
layout(location = 2) in vec3 normal;

out VertexData
{
   vec3 normal;
   vec3 view;
   vec2 texCoord;
}
outData;

void main()
{
    vec4 v_position = projection_matrix * modelview_matrix * vec4(position, 1.0);
    outData.normal = mat3(modelview_matrix) * normal;
    outData.view = -v_position.xyz;
	outData.texCoord = texCoord;
    gl_Position = v_position;
}
