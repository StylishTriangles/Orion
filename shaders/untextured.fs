#version 330 core
struct Light {
    vec3 position;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

in vec3 FragPos;
in vec3 Normal;

out vec4 color;

uniform vec3 color_ambient;
uniform vec3 color_diffuse;
uniform vec3 color_specular;
uniform float opacity;
uniform vec3 viewPos;
uniform float shininess;
uniform Light light;

void main()
{
//    color = texture2D(diffuseMap, TexCoord);
    // Ambient
    vec4 ambient = vec4(light.ambient, 0.333333333f) * vec4(color_ambient, 1);

    // Diffuse
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(light.position - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec4 diffuse = vec4(light.diffuse * diff, 0.333333333f) * vec4(color_diffuse, 1);

    // Specular
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = 0.5*pow(max(dot(viewDir, reflectDir), 0.0), shininess);
    vec4 specular = vec4(light.specular * spec, 0.333333333f) * vec4(color_specular, 1);

    color = ambient + diffuse + specular;
}
