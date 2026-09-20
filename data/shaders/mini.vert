#version 330

uniform mat4 modelview_matrix;
uniform mat4 projection_matrix;
uniform float pointSize;

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;

out VertexData {
	vec3 position;
	vec3 normal;
} OUT;

void main()
{
	vec4 vertexPosition = modelview_matrix * vec4(position, 1.0);
	OUT.position = vertexPosition.xyz / vertexPosition.w;
	OUT.normal = mat3(modelview_matrix) * normal;
	gl_Position = projection_matrix * vertexPosition;
	gl_PointSize = pointSize;
}