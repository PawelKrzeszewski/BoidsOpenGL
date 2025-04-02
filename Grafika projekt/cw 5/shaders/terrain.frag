#version 430 core

in vec3 fragPos;
in vec3 normal;
in float height;
in vec4 FragPosLightSpace;  // Otrzymane z vertex shader'a

out vec4 outColor;

uniform sampler2D shadowMap;  // Tekstura depth mapy (shadow map)

// Funkcja obliczaj�ca shadow mapping
float ShadowCalculation(vec4 fragPosLightSpace)
{
    // Podzia� perspektywiczny
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    // Przekszta�cenie z [-1, 1] do [0, 1]
    projCoords = projCoords * 0.5 + 0.5;
    // Pobranie zapisanej g��boko�ci (najbli�szej warto�ci) z depth mapy
    float closestDepth = texture(shadowMap, projCoords.xy).r;
    // Aktualna g��boko�� fragmentu (w przestrzeni �wiat�a)
    float currentDepth = projCoords.z;
    // Niewielki bias, aby zapobiec efektowi "shadow acne"
    float bias = 0.005;
    // Je�li aktualna g��boko��, pomniejszona o bias, jest wi�ksza ni� pobrana � fragment jest w cieniu
    float shadow = currentDepth - bias > closestDepth ? 1.0 : 0.0;
    return shadow;
}

void main() {
    vec3 baseColor = vec3(0.2, 0.5, 0.2);  // Zielona podstawa
    vec3 rockColor = vec3(0.5, 0.5, 0.5);    // Szary dla skalistych obszar�w

    // Mieszanie kolor�w w zale�no�ci od wysoko�ci
    float rockThreshold = 0.5;
    vec3 terrainColor = mix(baseColor, rockColor, smoothstep(0.0, rockThreshold, height));

    // Podstawowe o�wietlenie � u�ywamy sta�ego kierunku �wiat�a
    vec3 lightDir = normalize(vec3(0.0, -30.0, 0.0));
    float diffuse = max(dot(normal, lightDir), 0.0);

    // Obliczenie cienia
    float shadow = ShadowCalculation(FragPosLightSpace);

    // Ambient nie jest cieniowany, a diffuse jest pomniejszane przez shadow
    vec3 ambient = terrainColor * 0.3;
    vec3 diffuseColor = terrainColor * 0.7 * diffuse * (1.0 - shadow);

    outColor = vec4(ambient + diffuseColor, 1.0);
}
