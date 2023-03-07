#version 120

varying vec2 out_texture_coordinate;

uniform sampler2D u_occupancy;

void main()
{
  gl_FragColor = texture2D(u_occupancy, out_texture_coordinate);
  if (gl_FragColor.a == 0)
  {
    discard;
  }
}
