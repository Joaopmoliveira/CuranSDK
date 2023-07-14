#ifndef CURAN_TEMPLATED_VOLUME_ALGORITHMS_HEADER_FILE_
#define CURAN_TEMPLATED_VOLUME_ALGORITHMS_HEADER_FILE_

#include "ImageProcessingDefinitions.h"
#include "SplicingTools.h"
#include "KernelDescriptor.h"

namespace curan {
namespace image {
namespace reconstruction {
		
template<typename input_pixel_type,typename output_pixel_type,int ratio>
int TemplatedTrilinearInterpolation(const Eigen::Vector4d point,
	input_pixel_type* inPtr,
	output_pixel_type* outPtr,
	unsigned short* accPtr,
	int numscalars,
	Compounding compoundingMode,
	int outExt[6],
	uint64_t outInc[3],
	unsigned int* accOverflowCount)
{
	// Determine if the output is a floating point or integer type. If floating point type then we don't round
	// the interpolated value.
    constexpr bool needs_rounding = !std::is_floating_point<output_pixel_type>::value;

	double fx, fy, fz;

	// convert point[0] into integer component and a fraction
	int outIdX0 = std::floor(point[0]);
	fx = point[0] - outIdX0;
	// point[0] is unchanged, outIdX0 is the integer (floor), fx is the float
	int outIdY0 = std::floor(point[1]);
	fy = point[1] - outIdY0;
	int outIdZ0 = std::floor(point[2]);
	fz = point[2] - outIdZ0;

	int outIdX1 = outIdX0 + (fx != 0); // ceiling
	int outIdY1 = outIdY0 + (fy != 0);
	int outIdZ1 = outIdZ0 + (fz != 0);

	// at this point in time we have the floor (outIdX0), the ceiling (outIdX1)
	// and the fractional component (fx) for x, y and z

	// bounds check
	if ((outIdX0 | (outExt[1] - outExt[0] - outIdX1) |
		outIdY0 | (outExt[3] - outExt[2] - outIdY1) |
		outIdZ0 | (outExt[5] - outExt[4] - outIdZ1)) >= 0)
	{
		// do reverse trilinear interpolation
		uint64_t factX0 = outIdX0 * outInc[0];
		uint64_t factY0 = outIdY0 * outInc[1];
		uint64_t factZ0 = outIdZ0 * outInc[2];
		uint64_t factX1 = outIdX1 * outInc[0];
		uint64_t factY1 = outIdY1 * outInc[1];
		uint64_t factZ1 = outIdZ1 * outInc[2];
		
		uint64_t factY0Z0 = factY0 + factZ0;
		uint64_t factY0Z1 = factY0 + factZ1;
		uint64_t factY1Z0 = factY1 + factZ0;
		uint64_t factY1Z1 = factY1 + factZ1;

		// increment between the output pointer and the 8 pixels to work on
		uint64_t idx[8];
		idx[0] = factX0 + factY0Z0;
		idx[1] = factX0 + factY0Z1;
		idx[2] = factX0 + factY1Z0;
		idx[3] = factX0 + factY1Z1;
		idx[4] = factX1 + factY0Z0;
		idx[5] = factX1 + factY0Z1;
		idx[6] = factX1 + factY1Z0;
		idx[7] = factX1 + factY1Z1;

		// remainders from the fractional components - difference between the fractional value and the ceiling
		double rx = 1 - fx;
		double ry = 1 - fy;
		double rz = 1 - fz;

		double ryrz = ry * rz;
		double ryfz = ry * fz;
		double fyrz = fy * rz;
		double fyfz = fy * fz;

		double fdx[8]; // fdx is the weight towards the corner
		fdx[0] = rx * ryrz;
		fdx[1] = rx * ryfz;
		fdx[2] = rx * fyrz;
		fdx[3] = rx * fyfz;
		fdx[4] = fx * ryrz;
		fdx[5] = fx * ryfz;
		fdx[6] = fx * fyrz;
		fdx[7] = fx * fyfz;

		double f, r, a;
		input_pixel_type* inPtrTmp;
        output_pixel_type* outPtrTmp;

		unsigned short* accPtrTmp;

		// loop over the eight voxels
		int j = 8;
		do
		{
			j--;
			if (fdx[j] == 0)
			{
				continue;
			}
			inPtrTmp = inPtr;
			outPtrTmp = outPtr + idx[j];
			accPtrTmp = accPtr + ((idx[j] / outInc[0]));
			a = *accPtrTmp;

			int i = numscalars;
			do
			{
				i--;
				switch (compoundingMode)
				{
				case MAXIMUM_COMPOUNDING_MODE:
				{
					constexpr double minWeight = 0.125; // If a pixel is right in the middle of the eight surrounding voxels
					// (trilinear weight = 0.125 for each), then it the compounding operator
					// should be applied for each. Else, it should only be considered
					// for the other nearest voxels.
					if (fdx[j] >= minWeight && *inPtrTmp > *outPtrTmp)
					{
						if constexpr(ratio==1)
							*outPtrTmp = (*inPtrTmp);
						else
							*outPtrTmp = (*inPtrTmp)/(double)ratio;
						f = fdx[j];
						a = f * ACCUMULATION_MULTIPLIER;;
					}
					break;
				}
				case LATEST_COMPOUNDING_MODE:
				{
					const double minWeight(0.125); // If a pixel is right in the middle of the eight surrounding voxels
					// (trilinear weight = 0.125 for each), then it the compounding operator
					// should be applied for each. Else, it should only be considered
					// for the other nearest voxels.
					if (fdx[j] >= minWeight)
					{	
						if constexpr(ratio==1)
							*outPtrTmp = (*inPtrTmp);
						else
							*outPtrTmp = (*inPtrTmp)/(double)ratio;
						f = fdx[j];
						a = f * ACCUMULATION_MULTIPLIER;;
					}
					break;
				}
				case MEAN_COMPOUNDING_MODE:
					f = fdx[j];
					r = double((*accPtrTmp) / (double)ACCUMULATION_MULTIPLIER); // added division by double, since this always returned 0 otherwise
					a = f + r;
					if constexpr(ratio==1)
						*outPtrTmp = (needs_rounding) ? std::round((f * (*inPtrTmp) + r * (*outPtrTmp)) / a) : ((f * (*inPtrTmp) + r * (*outPtrTmp)) / a);
					else
						*outPtrTmp = (needs_rounding) ? std::round((f * (*inPtrTmp) + r * (*outPtrTmp)) / a)/(double)ratio : ((f * (*inPtrTmp) + r * (*outPtrTmp)) / a)/(double)ratio;
                    
					a *= ACCUMULATION_MULTIPLIER; // needs to be done for proper conversion to unsigned short for accumulation buffer
					break;
				default: //assume lattest which is cheap
					const double minWeight(0.125); // If a pixel is right in the middle of the eight surrounding voxels
					// (trilinear weight = 0.125 for each), then it the compounding operator
					// should be applied for each. Else, it should only be considered
					// for the other nearest voxels.
					if (fdx[j] >= minWeight)
					{
						if constexpr(ratio==1)
							*outPtrTmp = (*inPtrTmp);
						else
							*outPtrTmp = (*inPtrTmp)/(double)ratio;
						f = fdx[j];
						a = f * ACCUMULATION_MULTIPLIER;;
					}
					break;
					break;
				}
				inPtrTmp++;
				outPtrTmp++;
			} while (i); // number of scalars

			double newa = a;
			if (newa > ACCUMULATION_THRESHOLD && *accPtrTmp <= ACCUMULATION_THRESHOLD)
			{
				(*accOverflowCount) += 1;
			}

			// don't allow accumulation buffer overflow
			*accPtrTmp = ACCUMULATION_MAXIMUM;
			if (newa < ACCUMULATION_MAXIMUM)
			{
				// round the fixed point to the nearest whole unit, and save the result as an unsigned short into the accumulation buffer
				*accPtrTmp = std::round(newa);
			}
		} while (j);
		return 1;
	}
	// if bounds check fails
	return 0;
}


template<typename input_pixel_type,typename output_pixel_type,int ratio>
int TemplatedNearestNeighborInterpolation(const Eigen::Vector4d point,
	input_pixel_type* inPtr,
	output_pixel_type* outPtr,
	unsigned short* accPtr,
	int numscalars,
	Compounding compoundingMode,
	int outExt[6],
	uint64_t outInc[3],
	unsigned int* accOverflowCount)
{
	int i;
	// The nearest neighbor interpolation occurs here
	// The output point is the closest point to the input point - rounding
	// to get closest point
	int outIdX = std::round(point[0]) - outExt[0];
	int outIdY = std::round(point[1]) - outExt[2];
	int outIdZ = std::round(point[2]) - outExt[4];

	// fancy way of checking bounds
	if ((outIdX | (outExt[1] - outExt[0] - outIdX) |
		outIdY | (outExt[3] - outExt[2] - outIdY) |
		outIdZ | (outExt[5] - outExt[4] - outIdZ)) >= 0)
	{
		int inc = outIdX * outInc[0] + outIdY * outInc[1] + outIdZ * outInc[2];
		outPtr += inc;
		switch (compoundingMode)
		{
		case (MAXIMUM_COMPOUNDING_MODE):
		{
			accPtr += inc / outInc[0];

			int newa = *accPtr + ACCUMULATION_MULTIPLIER;
			if (newa > ACCUMULATION_THRESHOLD)
			{
				(*accOverflowCount) += 1;
			}

			for (i = 0; i < numscalars; i++)
			{
				if (*inPtr > *outPtr)
				{
					if constexpr(ratio==1)
						*outPtr = (*inPtr);
					else
						*outPtr = (*inPtr)/(double)ratio;
				}
				inPtr++;
				outPtr++;
			}

			*accPtr = ACCUMULATION_MAXIMUM; // set to 0xFFFF by default for overflow protection
			if (newa < ACCUMULATION_MAXIMUM)
			{
				*accPtr = newa;
			}

			break;
		}
		case (MEAN_COMPOUNDING_MODE):
		{
			accPtr += inc / outInc[0];
			if (*accPtr <= ACCUMULATION_THRESHOLD)   // no overflow, act normally
			{
				int newa = *accPtr + ACCUMULATION_MULTIPLIER;
				if (newa > ACCUMULATION_THRESHOLD)
				{
					(*accOverflowCount) += 1;
				}

				for (i = 0; i < numscalars; i++)
				{
					if constexpr(ratio==1)
						*outPtr = (((*inPtr++) * ACCUMULATION_MULTIPLIER + (*outPtr) * (*accPtr)) / newa);
					else
						*outPtr = (((*inPtr++) * ACCUMULATION_MULTIPLIER + (*outPtr) * (*accPtr)) / newa)/(double)ratio;
					outPtr++;
				}

				*accPtr = ACCUMULATION_MAXIMUM; // set to 0xFFFF by default for overflow protection
				if (newa < ACCUMULATION_MAXIMUM)
				{
					*accPtr = newa;
				}
			} else     // overflow, use recursive filtering with 255/256 and 1/256 as the weights, since 255 voxels have been inserted so far
			{ 
				if constexpr(ratio==1)
					*outPtr = (char_pixel_type)(fraction1_256 * (*inPtr++) + fraction255_256 * (*outPtr));
				else{
					*outPtr = (char_pixel_type)(fraction1_256 * ((*inPtr/(double)ratio)) + fraction255_256 * (*outPtr));
					++inPtr;
				}
				// TODO: Should do this for all the scalars, and accumulation?
				
			}
			break;
		}
		case (LATEST_COMPOUNDING_MODE):
		default:
		{
			accPtr += inc / outInc[0];

			int newa = *accPtr + ACCUMULATION_MULTIPLIER;
			if (newa > ACCUMULATION_THRESHOLD)
			{
				(*accOverflowCount) += 1;
			}

			for (i = 0; i < numscalars; i++)
			{
				if constexpr(ratio==1)
					*outPtr = (*inPtr);
				else
					*outPtr = (*inPtr)/(double)ratio;
				inPtr++;
				outPtr++;
			}

			*accPtr = ACCUMULATION_MAXIMUM; // set to 0xFFFF by default for overflow protection
			if (newa < ACCUMULATION_MAXIMUM)
			{
				*accPtr = newa;
			}

			break;
		}
		}
		return 1;
	}
	return 0;
}

template<typename input_pixel_type,typename output_pixel_type>
struct PasteSliceIntoVolumeInsertSliceParamsTemplated
{
	// information on the volume
	std::array<double,3> out_spacing;
	std::array<size_t,3> out_size;
	std::array<size_t,3> out_origin;
	output_pixel_type* outPtr;
	unsigned short* accPtr; 
	std::array<double,3> in_spacing;
	std::array<size_t,3> in_size;
	std::array<double,3> in_origin;
	input_pixel_type* inPtr; 
	int* inExt; 
	unsigned int* accOverflowCount;

	Eigen::Matrix4d matrix;

	Interpolation interpolationMode;   // linear or nearest neighbor
	Compounding compoundingMode;

	// parameters for clipping
	double* clipRectangleOrigin; // array size 2
	double* clipRectangleSize; // array size 2

	double pixelRejectionThreshold;
	int image_number;
};

template<typename input_pixel_type,typename output_pixel_type,int ratio>
void TemplatedUnoptimizedInsertSlice(PasteSliceIntoVolumeInsertSliceParamsTemplated<input_pixel_type,output_pixel_type>* insertionParams){
	// information on the volume
	output_pixel_type* outPtr = insertionParams->outPtr;
	unsigned short* accPtr = insertionParams->accPtr;
	input_pixel_type* inPtr = insertionParams->inPtr;
	int* inExt = insertionParams->inExt;
	unsigned int* accOverflowCount = insertionParams->accOverflowCount;

	// details specified by the user RE: how the voxels should be computed
	Interpolation interpolationMode = insertionParams->interpolationMode;   // linear or nearest neighbor
	Compounding compoundingMode = insertionParams->compoundingMode;         // weighted average or maximum

	// parameters for clipping
	double* clipRectangleOrigin = insertionParams->clipRectangleOrigin; // array size 2
	double* clipRectangleSize = insertionParams->clipRectangleSize; // array size 2

	// slice spacing and origin
	double inSpacing[3];
	inSpacing[0] = insertionParams->in_spacing[0];
	inSpacing[1] = insertionParams->in_spacing[1];
	inSpacing[2] = insertionParams->in_spacing[2];

	double inOrigin[3];
	inOrigin[0] = insertionParams->in_origin[0];
	inOrigin[1] = insertionParams->in_origin[1];
	inOrigin[2] = insertionParams->in_origin[2];

	// get the clip rectangle as an extent
	int clipExt[6];
	GetClipExtent(clipExt, inOrigin, inSpacing, inExt, clipRectangleOrigin, clipRectangleSize);

	// find maximum output range = output extent
	int outExt[6];
	double outSpacing[3];
	outSpacing[0] = insertionParams->out_spacing[0];
	outSpacing[1] = insertionParams->out_spacing[1];
	outSpacing[2] = insertionParams->out_spacing[2];

	outExt[0] = 0;
	outExt[1] = insertionParams->out_size[0] - 1;

	outExt[2] = 0;
	outExt[3] = insertionParams->out_size[1] - 1;

	outExt[4] = 0;
	outExt[5] = insertionParams->out_size[2] - 1;

	// Get increments to march through data - ex move from the end of one x scanline of data to the
	// start of the next line
	uint64_t outInc[3] = { 0,0,0 };

	uint64_t  incr = INPUT_COMPONENTS;
	for (int i = 0; i < 3; ++i) {
		outInc[i] = incr;
		incr *= (outExt[i * 2 + 1] - outExt[i * 2] + 1);
	}

	uint64_t inIncX = 0, inIncY = 0, inIncZ = 0;

	// Get increments to march through data - ex move from the end of one x scanline of data to the
	// start of the next line
	uint64_t inInc[3] = { 0,0,0 };

	incr = INPUT_COMPONENTS;
	for (int i = 0; i < 3; ++i) {
		inInc[i] = incr;
		incr *= (inExt[i * 2 + 1] - inExt[i * 2] + 1);
	}

	int e0, e1, e2, e3;

	inIncX = 0;

	e0 = inExt[0];
	e1 = inExt[1];
	e2 = inExt[2];
	e3 = inExt[3];

	inIncY = inInc[1] - (e1 - e0 + 1) * inInc[0];
	inIncZ = inInc[2] - (e3 - e2 + 1) * inInc[1];

	int numscalars = INPUT_COMPONENTS;

	// Loop through  slice pixels in the input extent and put them into the output volume
	// the resulting point in the output volume (outPoint) from a point in the input slice
	// (inpoint)
	Eigen::Vector4d inPoint;
	Eigen::Vector4d outPoint;
	inPoint[3] = 1;

	//we need to properly offset the data to the correct position due to the influence of the multithreading aspect
    //   0 1 2 3 4 5 6 7 8 9 10 11 12 13   14 15 16 17 18 19 20 21 22 23 24 25 26 27    28 
	// [ x x x x x x x x x x  x  x  x  x] [x  x   x  x  x  x  x  x  x  x  x  x  x  x] [ x x x x x x x x x x x x x x]
	// lets think y and x first
	// offset = inExt[0] (the x offset (should be null)) + inExt[2]*size_in[0] (we shift the extent to the line of x which corresponds to us) + inExt[4]*size_
	inPtr += inExt[0]+inExt[2]*insertionParams->in_size[0]+inExt[4]*insertionParams->in_size[0]*insertionParams->in_size[1];
	switch(interpolationMode){
	case LINEAR_INTERPOLATION:
	for (int idZ = inExt[4]; idZ <= inExt[5]; idZ++, inPtr += inIncZ)
	{
		for (int idY = inExt[2]; idY <= inExt[3]; idY++, inPtr += inIncY)
		{
			for (int idX = inExt[0]; idX <= inExt[1]; idX++, inPtr += numscalars)
			{
				// check if we are within the current clip extent
				if (idX < clipExt[0] || idX > clipExt[1] || idY < clipExt[2] || idY > clipExt[3])
				{
					continue;
				}
				//scale the input from pixels to mm 
				inPoint[0] = idX * inSpacing[0];
				inPoint[1] = idY * inSpacing[1];
				inPoint[2] = idZ * inSpacing[2];

				//transform the point into the output coordinates
				outPoint = insertionParams->matrix * inPoint;

				//scale the output from milimiters to pixels
				outPoint[0] /= outSpacing[0];
				outPoint[1] /= outSpacing[1];
				outPoint[2] /= outSpacing[2];
				outPoint[3] = 1;
			
				// interpolation functions return 1 if the interpolation was successful, 0 otherwise
				TemplatedTrilinearInterpolation<input_pixel_type,output_pixel_type,ratio>(outPoint, inPtr, outPtr, accPtr, numscalars, compoundingMode, outExt, outInc, accOverflowCount);
			}
		}
	}
	break;
	case NEAREST_NEIGHBOR_INTERPOLATION:
	default: 
	for (int idZ = inExt[4]; idZ <= inExt[5]; idZ++, inPtr += inIncZ)
	{
		for (int idY = inExt[2]; idY <= inExt[3]; idY++, inPtr += inIncY)
		{
			for (int idX = inExt[0]; idX <= inExt[1]; idX++, inPtr += numscalars)
			{
				// check if we are within the current clip extent
				if (idX < clipExt[0] || idX > clipExt[1] || idY < clipExt[2] || idY > clipExt[3])
				{
					continue;
				}
				//scale the input from pixels to mm 
				inPoint[0] = idX * inSpacing[0];
				inPoint[1] = idY * inSpacing[1];
				inPoint[2] = idZ * inSpacing[2];

				//transform the point into the output coordinates
				outPoint = insertionParams->matrix * inPoint;

				//scale the output from milimiters to pixels
				outPoint[0] /= outSpacing[0];
				outPoint[1] /= outSpacing[1];
				outPoint[2] /= outSpacing[2];
				outPoint[3] = 1;
			
				// interpolation functions return 1 if the interpolation was successful, 0 otherwise
				TemplatedNearestNeighborInterpolation<input_pixel_type,output_pixel_type,ratio>(outPoint, inPtr, outPtr, accPtr, numscalars, compoundingMode, outExt, outInc, accOverflowCount);
			}
		}
	}
	break;
	};

}

}
}
}

#endif