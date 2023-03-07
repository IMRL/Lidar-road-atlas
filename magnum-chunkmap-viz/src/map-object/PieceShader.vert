// #version 330

layout(location = 0) in vec4 position;
layout(location = 1) in vec2 textureCoordinates;

uniform mat4 transformationMatrix;
uniform mat4 projectionMatrix;

varying vec2 out_texture_coordinate;

uniform sampler2D u_elevation;
// uniform float u_elevation_alpha;
// uniform float u_elevation_beta;

void main()
{
    out_texture_coordinate = vec2(textureCoordinates);
    vec4 pos = position;
    pos.z += texture(u_elevation, textureCoordinates.st).x;// * u_elevation_alpha + u_elevation_beta;
    gl_Position = projectionMatrix * transformationMatrix * pos;
}
