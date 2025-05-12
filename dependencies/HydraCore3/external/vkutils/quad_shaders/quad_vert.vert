#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform params_t
{
  vec4 scaleAndOffs;
  vec4 depthMinMaxScale;

} params;

layout (location = 0 ) out VS_OUT
{
  vec2 texCoord;
} vOut;

void main(void)
{
  if(gl_VertexIndex == 0)
  {
    gl_Position   = vec4(-1.0f*params.scaleAndOffs.x + params.scaleAndOffs.z, -1.0f*params.scaleAndOffs.y + params.scaleAndOffs.w, 0.0f, 1.0f);
    vOut.texCoord = vec2(0.0f, 0.0f);
  }
  else if(gl_VertexIndex == 1) 
  {
    gl_Position   = vec4(-1.0f*params.scaleAndOffs.x + params.scaleAndOffs.z, +1.0f*params.scaleAndOffs.y + params.scaleAndOffs.w, 0.0f, 1.0f);
    vOut.texCoord = vec2(0.0f, 1.0f);
  }
  else if(gl_VertexIndex == 2) 
  {
    gl_Position   = vec4(+1.0f*params.scaleAndOffs.x + params.scaleAndOffs.z, -1.0f*params.scaleAndOffs.y + params.scaleAndOffs.w, 0.0f, 1.0f);
    vOut.texCoord = vec2(1.0f, 0.0f);
  }
  else
  { 
    gl_Position   = vec4(+1.0f*params.scaleAndOffs.x + params.scaleAndOffs.z, +1.0f*params.scaleAndOffs.y + params.scaleAndOffs.w, 0.0f, 1.0f);
    vOut.texCoord = vec2(1.0f, 1.0f);
  }

  gl_Position.y = -gl_Position.y;	// Vulkan coordinate system is different t OpenGL    
}
