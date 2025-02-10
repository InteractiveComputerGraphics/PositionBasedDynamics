#version 330

uniform mat4 modelview_matrix;

layout(location = 0) in vec3 position;
layout(location = 1) in vec2 texCoord;

out VertexData
{
   vec3 view;
   vec2 texCoord;
}
outData;

void main()
{
    vec4 v_position = modelview_matrix * vec4(position, 1.0);
    outData.view = -v_position.xyz;
	outData.texCoord = texCoord;
    gl_Position = v_position;
}
