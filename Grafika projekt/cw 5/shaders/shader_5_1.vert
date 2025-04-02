#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 transformation;
uniform mat4 lightSpaceMatrix;
uniform mat4 model;

out vec3 FragPos;
out vec3 Normal;
out vec4 FragPosLightSpace;

void main()
{
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = transpose(inverse(mat3(model))) * aNormal;
    FragPosLightSpace = lightSpaceMatrix * vec4(FragPos, 1.0);
    gl_Position = transformation * vec4(aPos, 1.0);
}