#version 330

uniform vec3 surface_color;
uniform float shininess;
uniform float specular_factor;
uniform sampler2D tex;

layout(location = 0, index = 0) out vec4 frag_color;

in VertexData
{
    vec3 normal;
    vec3 view;
	vec2 texCoord;
}
inData;

void main()
{
    vec3 normal = inData.normal;

    if (!gl_FrontFacing)
    {
        normal = -inData.normal;
    }
	
    const vec3 ambient = vec3(0.6, 0.6, 0.6);
    const vec3 diffuse = vec3(1.0, 1.0, 1.0);
    const vec3 specular = vec3(1.0, 1.0, 1.0);	
	
	vec3 eye_n = normalize(normal);
	vec3 eye_v = normalize(inData.view);
    const vec3 eye_l = vec3(0.0, 0.0, 1.0);	
    float cos_ln = max(dot(eye_l, eye_n), 0.0);    
    vec3 h = normalize(eye_l + eye_v);
    float cos_nh = max(dot(eye_n, h), 0.0);   
    
	vec3 color = surface_color * (ambient + diffuse * cos_ln + specular_factor * specular * pow(cos_nh, shininess));
	vec4 color2 = texture2D(tex, inData.texCoord) * vec4(color, 1.0);

    frag_color = color2;
}
