#version 450 core
in vec4 outColor;
out vec4 FragColor;

void main()
{
    FragColor = vec4(outColor);
} 