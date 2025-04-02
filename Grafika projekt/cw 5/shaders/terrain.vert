#version 430 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

uniform mat4 viewProjectionMatrix;
uniform mat4 lightSpaceMatrix; 

out vec3 fragPos;
out vec3 normal;
out float height;
out vec4 FragPosLightSpace;   // Wyj�cie � pozycja w przestrzeni �wiat�a

void main() {
    fragPos = aPos;
    normal = aNormal;
    height = aPos.y;
    // Obliczenie pozycji w przestrzeni �wiat�a
    FragPosLightSpace = lightSpaceMatrix * vec4(aPos, 1.0);
    gl_Position = viewProjectionMatrix * vec4(aPos, 1.0);
}
