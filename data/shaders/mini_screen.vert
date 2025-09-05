#version 330

uniform float width;
uniform float height;

layout(location = 0) in vec2 position;

void main()
{
	// Project the screen position to NDC (cf. glOrtho).
	gl_Position = vec4(2.0 * (position.x / width) - 1.0, -2.0 * (position.y / height) + 1.0, -1.0, 1.0);
}