// Orthographic RGB/depth z-buffer. Coordinates remain metric; no hole filling.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

extern "C" void fd_raster(const uint8_t* rgb, const uint16_t* depth,
                         const float* rays, int samples, const float* position,
                         const uint8_t* region, int width, int height,
                         float xmax, float ymax, float resolution,
                         uint8_t* output, float* zbuffer) {
  std::fill(output, output+width*height*3, uint8_t(0));
  std::fill(zbuffer, zbuffer+width*height, -std::numeric_limits<float>::infinity());
  for (int i=0; i<samples; ++i) {
    if (depth[i]<300 || depth[i]>10000) continue;
    const float distance=depth[i]*.001f;
    const float x=rays[3*i]*distance+position[0];
    const float y=rays[3*i+1]*distance+position[1];
    const float z=rays[3*i+2]*distance+position[2];
    if (!(z>-.2f && z<2.f)) continue;
    const int row=static_cast<int>(std::floor((xmax-x)/resolution));
    const int col=static_cast<int>(std::floor((ymax-y)/resolution));
    if (row<0 || row>=height || col<0 || col>=width) continue;
    const int cell=row*width+col;
    if (!region[cell] || z<zbuffer[cell]) continue;
    zbuffer[cell]=z;
    std::copy(rgb+3*i,rgb+3*i+3,output+3*cell);
  }
}
