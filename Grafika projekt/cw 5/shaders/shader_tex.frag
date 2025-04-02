#version 430 core

uniform vec3 lightPos;
uniform sampler2D colorTexture;
uniform sampler2D normalMap;

in vec3 fragPos;
in vec2 texCoords;
in mat3 TBN;

out vec4 outColor;

void main()
{
    vec3 lightDir = normalize(lightPos - fragPos);
    
    // Sample normal from normal map and convert to [-1,1] range
    vec3 normal = texture(normalMap, texCoords).rgb * 2.0 - 1.0;

    // Transform normal from tangent space to world space
    normal = normalize(TBN * normal);

    // Sample diffuse texture
    vec3 textureColor = texture(colorTexture, texCoords).rgb;

    // Compute diffuse lighting
    float diffuse = max(dot(normal, lightDir), 0.0);

    outColor = vec4(textureColor * diffuse, 1.0);
}
