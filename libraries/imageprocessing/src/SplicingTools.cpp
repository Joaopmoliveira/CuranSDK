#include "imageprocessing/SplicingTools.h"
#include <cmath>
#include <cstring>

namespace curan{
namespace image {

void GetClipExtent(int clipExt[6],
	double inOrigin[3],
	double inSpacing[3],
	const int inExt[6],
	double clipRectangleOrigin[2],
	double clipRectangleSize[2])
{
	// Map the clip rectangle (millimetres) to pixels
	// --> number of pixels (+ or -) from the origin

	int x0 = (int)std::ceil(clipRectangleOrigin[0]);
	int x1 = (int)std::floor(clipRectangleOrigin[0] + clipRectangleSize[0]);
	int y0 = (int)std::ceil(clipRectangleOrigin[1]);
	int y1 = (int)std::floor(clipRectangleOrigin[1] + clipRectangleSize[1]);

	// Make sure that x0 <= x1 and y0 <= y1
	if (x0 > x1)
	{
		int tmp = x0;
		x0 = x1;
		x1 = tmp;
	}
	if (y0 > y1)
	{
		int tmp = y0;
		y0 = y1;
		y1 = tmp;
	}

	// make sure the clip extent lies within the input extent
	if (x0 < inExt[0])
	{
		x0 = inExt[0];
	}
	if (x1 > inExt[1])
	{
		x1 = inExt[1];
	}
	// clip extent was outside of range of input extent
	if (x0 > x1)
	{
		x0 = inExt[0];
		x1 = inExt[0] - 1;
	}

	if (y0 < inExt[2])
	{
		y0 = inExt[2];
	}
	if (y1 > inExt[3])
	{
		y1 = inExt[3];
	}
	// clip extent was outside of range of input extent
	if (y0 > y1)
	{
		y0 = inExt[2];
		y1 = inExt[2] - 1;
	}

	// Set the clip extent
	clipExt[0] = x0;
	clipExt[1] = x1;
	clipExt[2] = y0;
	clipExt[3] = y1;
	clipExt[4] = inExt[4];
	clipExt[5] = inExt[5];
}

bool splice_input_extent( std::vector<std::array<int,6>>& splitting, const int fullExt[6]){
	size_t thread_id = 0;
	for(auto& nsplit : splitting){
		int min, max;
  		std::memcpy( nsplit.data(), fullExt, 6 * sizeof( int ) );
  		int splitAxis = 2; 
  		min = fullExt[4];
  		max = fullExt[5];
  		while ( min == max ){
    		--splitAxis;
    		if ( splitAxis < 0 ){
				//return failure
      			return false;
			}
			min = fullExt[splitAxis * 2];
    		max = fullExt[splitAxis * 2 + 1];
   		}
  		int range = max - min + 1;
  		int valuesPerThread = ( int )std::ceil( range / ( double )splitting.size() );
 		int maxThreadIdUsed = ( int )std::ceil( range / ( double )valuesPerThread ) - 1;
  		if ( thread_id < maxThreadIdUsed ){
    		nsplit[splitAxis * 2] = nsplit[splitAxis * 2] + thread_id * valuesPerThread;
    		nsplit[splitAxis * 2 + 1] = nsplit[splitAxis * 2] + valuesPerThread - 1;
		}
  		if ( thread_id == maxThreadIdUsed )
			nsplit[static_cast<std::array<int,6>::size_type>(splitAxis) * 2] = nsplit[splitAxis * 2] + thread_id * valuesPerThread;
		++thread_id;
	}
  	return true;
}

}
}