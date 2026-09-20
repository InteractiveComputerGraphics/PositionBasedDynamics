#version 330

uniform vec3 color;

out vec4 outColor;

void main()
{
	outColor = vec4(color, 1.0);
}