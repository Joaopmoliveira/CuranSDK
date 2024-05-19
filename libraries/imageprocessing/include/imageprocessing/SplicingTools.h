#include <vector>
#include <array>

namespace curan{
namespace image {

/*
Convert the ClipRectangle into a clip extent that can be applied to the
input data - number of pixels (+ or -) from the origin (the z component
is copied from the inExt parameter)
\param clipExt {x0, x1, y0, y1, z0, z1} the "output" of this function is to change this array
\param inOrigin = {x, y, z} the origin in mm
\param inSpacing = {x, y, z} the spacing in mm
\param inExt = {x0, x1, y0, y1, z0, z1} min/max possible extent, in pixels
\param clipRectangleOrigin = {x, y} origin of the clipping rectangle in the image, in pixels
\param clipRectangleSize = {x, y} size of the clipping rectangle in the image, in pixels
*/
void GetClipExtent(int clipExt[6],
double inOrigin[3],
double inSpacing[3],
const int inExt[6],
double clipRectangleOrigin[2],
double clipRectangleSize[2]);

[[nodiscard]] bool splice_input_extent( std::vector<std::array<int,6>>& splitting, const int fullExt[6]);

}
}