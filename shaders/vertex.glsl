#version 450 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec4 aColor;

out vec4 outColor;

uniform mat4 modelMat;
uniform mat4 viewMat;
uniform mat4 projectionMat;

void main()
{
    gl_Position = projectionMat * viewMat * modelMat * vec4(aPos.x, aPos.y, aPos.z, 1.0);
    outColor = aColor;
}