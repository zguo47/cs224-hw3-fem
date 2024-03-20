#version 330 core
out vec4 fragColor;

// Additional information for lighting
in vec4 normal_worldSpace;
in vec4 position_worldSpace;

uniform int wire = 0;
uniform float red = 1.0;
uniform float green = 1.0;
uniform float blue = 1.0;
uniform float alpha = 1.0;

uniform float time;

void main() {
    if (wire == 1) {
        fragColor = vec4(0.0, 0.0, 0.0, 1);
        return;
    }
    vec4 lightPos   = vec4(-2.0, 2.0, -3.0 , 1.0);
    vec3 lightColor = vec3(1.0f, alpha, 0.0f);
    vec4 lightDir   = normalize(-lightPos + position_worldSpace);
    float c = clamp(dot(-normal_worldSpace, lightDir), 0, 1);
    float intensity = clamp(dot(-normal_worldSpace, lightDir), 0, 1);

    // Light 2
    vec4 lightPos2 = vec4(2.0, 2.0, -3.0, 1.0);
    vec3 lightColor2 = vec3(0.0f, 0.0f, 1.0f);
    vec4 lightDir2 = normalize(-lightPos2 + position_worldSpace);
    float intensity2 = clamp(dot(-normal_worldSpace, lightDir2), 0, 1);

    // Light 3
    vec4 lightPos3 = vec4(0.0, 2.0, 3.0, 1.0);
    vec3 lightColor3 = vec3(1.0f, 1.0f, 1.0f);
    vec4 lightDir3 = normalize(-lightPos3 + position_worldSpace);
    float intensity3 = clamp(dot(-normal_worldSpace, lightDir3), 0, 1);

    // Light 4
    vec4 lightPos4 = vec4(-2.0, 2.0, 3.0, 1.0);
    vec3 lightColor4 = vec3(0.5f, 0.5f, 0.0f);
    vec4 lightDir4 = normalize(-lightPos4 + position_worldSpace);
    float intensity4 = clamp(dot(-normal_worldSpace, lightDir4), 0, 1);

    // float blinkingIntensity = sin(time * 2.0) * 0.5 + 0.5;
    // lightColor *= blinkingIntensity;
    // lightColor2 *= blinkingIntensity;
    // lightColor3 *= blinkingIntensity;
    // lightColor4 *= blinkingIntensity;
    // Calculate color gradients for each light
    vec3 gradientColor1 = vec3(sin(time * 1.5) * 0.5 + 0.5, cos(time * 1.2) * 0.5 + 0.5, sin(time * 1.8) * 0.5 + 0.5);
    vec3 gradientColor2 = vec3(cos(time * 1.6) * 0.5 + 0.5, sin(time * 1.4) * 0.5 + 0.5, cos(time * 1.9) * 0.5 + 0.5);
    vec3 gradientColor3 = vec3(sin(time * 1.7) * 0.5 + 0.5, cos(time * 1.3) * 0.5 + 0.5, sin(time * 2.0) * 0.5 + 0.5);
    vec3 gradientColor4 = vec3(cos(time * 1.8) * 0.5 + 0.5, sin(time * 1.5) * 0.5 + 0.5, cos(time * 1.7) * 0.5 + 0.5);

    // Apply the gradients to the light colors
    lightColor *= gradientColor1;
    lightColor2 *= gradientColor2;
    lightColor3 *= gradientColor3;
    lightColor4 *= gradientColor4;

    // Combine the lighting contributions from each light source
    vec3 totalLight = lightColor * intensity + lightColor2 * intensity2 + lightColor3 * intensity3 + lightColor4 * intensity4;

    fragColor = vec4(red * totalLight[0], green * totalLight[1], blue * totalLight[2], 1);
    // fragColor = vec4(red * c * lightColor[0], green * c * lightColor[0], blue * c * lightColor[0], 1);
}
