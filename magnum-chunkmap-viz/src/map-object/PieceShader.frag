// #version 330

in vec2 out_texture_coordinate;

out vec4 fragColor;

uniform sampler2D u_occupancy;
uniform sampler2D u_colormap;

void main()
{
    fragColor = texture(u_occupancy, out_texture_coordinate);
    fragColor = vec4(texture(u_colormap, vec2(fragColor.r, 0.5)).rgb, fragColor.g);
    if (fragColor.a == 0)
    {
        discard;
    }
}
