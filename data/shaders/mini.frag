#version 330

#define NUM_LIGHTS 3

uniform bool lighting;
uniform vec3 ambientIntensity;
uniform vec3 diffuseIntensity[NUM_LIGHTS];
uniform vec3 specularIntensity[NUM_LIGHTS];
uniform vec3 lightPosition[NUM_LIGHTS];
uniform vec3 ambientReflectance;
uniform vec3 diffuseReflectance;
uniform vec3 specularReflectance;
uniform float shininess;

in VertexData {
	vec3 position;
	vec3 normal;
} IN;

out vec4 outColor;

void main()
{
	if (lighting)
	{
		vec3 view = normalize(-IN.position);
		vec3 normal = normalize(IN.normal);

		// Compute the ambient lighting.
		vec3 color = ambientReflectance * ambientIntensity;

		for (int i = 0; i < NUM_LIGHTS; i++)
		{
			vec3 light = normalize(lightPosition[i] - IN.position);
			float diffuseFactor = max(dot(normal, light), 0.0);
			if (diffuseFactor > 0.0)
			{
				// Compute the diffuse lighting using the Lambertian model.
				color += diffuseFactor * diffuseReflectance * diffuseIntensity[i];

				vec3 halfway = normalize(light + view);
				float specularFactor = pow(max(dot(halfway, normal), 0.0), shininess);

				// Compute the specular lighting using the Blinn-Phong model.
				color += specularFactor * specularReflectance * specularIntensity[i];
			}
		}

		outColor = vec4(color, 1.0);
	}
	else outColor = vec4(diffuseReflectance, 1.0);
}