#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 aTexCoords;

out vec3 FragPos;
out vec3 Normal;
out vec2 TexCoords;

uniform mat4 vp; // projection * view
uniform mat4 modelMat;
uniform mat4 modelNorm;

void main()
{
    TexCoords = aTexCoords;   
    gl_Position = vp * modelMat * vec4(position, 1.0f);
    FragPos = vec3(modelMat * vec4(position, 1.0f));
    Normal = mat3(modelNorm)*normal;
}
