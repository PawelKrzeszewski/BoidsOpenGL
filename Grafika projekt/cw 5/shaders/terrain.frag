#version 430 core

in vec3 fragPos;
in vec3 normal;
in float height;
in vec4 FragPosLightSpace;  // Otrzymane z vertex shader'a

out vec4 outColor;

uniform sampler2D shadowMap;  // Tekstura depth mapy (shadow map)

// Funkcja obliczaj¹ca shadow mapping
float ShadowCalculation(vec4 fragPosLightSpace)
{
    // Podzia³ perspektywiczny
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    // Przekszta³cenie z [-1, 1] do [0, 1]
    projCoords = projCoords * 0.5 + 0.5;
    // Pobranie zapisanej g³êbokoœci (najbli¿szej wartoœci) z depth mapy
    float closestDepth = texture(shadowMap, projCoords.xy).r;
    // Aktualna g³êbokoœæ fragmentu (w przestrzeni œwiat³a)
    float currentDepth = projCoords.z;
    // Niewielki bias, aby zapobiec efektowi "shadow acne"
    float bias = 0.005;
    // Jeœli aktualna g³êbokoœæ, pomniejszona o bias, jest wiêksza ni¿ pobrana – fragment jest w cieniu
    float shadow = currentDepth - bias > closestDepth ? 1.0 : 0.0;
    return shadow;
}

void main() {
    vec3 baseColor = vec3(0.2, 0.5, 0.2);  // Zielona podstawa
    vec3 rockColor = vec3(0.5, 0.5, 0.5);    // Szary dla skalistych obszarów

    // Mieszanie kolorów w zale¿noœci od wysokoœci
    float rockThreshold = 0.5;
    vec3 terrainColor = mix(baseColor, rockColor, smoothstep(0.0, rockThreshold, height));

    // Podstawowe oœwietlenie – u¿ywamy sta³ego kierunku œwiat³a
    vec3 lightDir = normalize(vec3(0.0, -30.0, 0.0));
    float diffuse = max(dot(normal, lightDir), 0.0);

    // Obliczenie cienia
    float shadow = ShadowCalculation(FragPosLightSpace);

    // Ambient nie jest cieniowany, a diffuse jest pomniejszane przez shadow
    vec3 ambient = terrainColor * 0.3;
    vec3 diffuseColor = terrainColor * 0.7 * diffuse * (1.0 - shadow);

    outColor = vec4(ambient + diffuseColor, 1.0);
}
