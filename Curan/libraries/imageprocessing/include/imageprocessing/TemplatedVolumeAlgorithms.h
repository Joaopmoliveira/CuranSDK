#ifndef CURAN_VOLUME_ALGORITHMS_HEADER_FILE_
#define CURAN_VOLUME_ALGORITHMS_HEADER_FILE_

#include "ImageProcessingDefinitions.h"
#include "KernelDescriptor.h"

namespace curan {
namespace image {
namespace reconstruction {
			
void GetClipExtentTemplate(int clipExt[6],
double inOrigin[3],
double inSpacing[3],
const int inExt[6],
double clipRectangleOrigin[2],
double clipRectangleSize[2]);

template<typename input_pixel_type,typename output_pixel_type,output_pixel_type convertion_ratio>
int TrilinearInterpolation(const Eigen::Vector4d point,
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
		input_pixel_type* inPtrTmp, 
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
						*outPtrTmp = (*inPtrTmp)/convertion_ratio;
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
						*outPtrTmp = (*inPtrTmp)/convertion_ratio;
						f = fdx[j];
						a = f * ACCUMULATION_MULTIPLIER;;
					}
					break;
				}
				case MEAN_COMPOUNDING_MODE:
					f = fdx[j];
					r = double((*accPtrTmp) / (double)ACCUMULATION_MULTIPLIER); // added division by double, since this always returned 0 otherwise
					a = f + r;
                    *outPtrTmp = (needs_rounding) ? std::round((f * (*inPtrTmp) + r * (*outPtrTmp)) / a)/convertion_ratio : ((f * (*inPtrTmp) + r * (*outPtrTmp)) / a)/convertion_ratio;
					a *= ACCUMULATION_MULTIPLIER; // needs to be done for proper conversion to unsigned short for accumulation buffer
					break;
				default:
					throw std::runtime_error("Undefined compounding mode");
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


template<typename input_pixel_type,typename output_pixel_type,output_pixel_type convertion_ratio>
int NearestNeighborInterpolation(const Eigen::Vector4d point,
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
					*outPtr = (*inPtr)/convertion_ratio;
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
					*outPtr = (((*inPtr++) * ACCUMULATION_MULTIPLIER + (*outPtr) * (*accPtr)) / newa)/convertion_ratio;
					outPtr++;
				}

				*accPtr = ACCUMULATION_MAXIMUM; // set to 0xFFFF by default for overflow protection
				if (newa < ACCUMULATION_MAXIMUM)
				{
					*accPtr = newa;
				}
			} else     // overflow, use recursive filtering with 255/256 and 1/256 as the weights, since 255 voxels have been inserted so far
			{ 
				// TODO: Should do this for all the scalars, and accumulation?
				*outPtr = (char_pixel_type)(fraction1_256 * (*inPtr++) + fraction255_256 * (*outPtr));
			}
			break;
		}
		case (LATEST_COMPOUNDING_MODE):
		{
			accPtr += inc / outInc[0];

			int newa = *accPtr + ACCUMULATION_MULTIPLIER;
			if (newa > ACCUMULATION_THRESHOLD)
			{
				(*accOverflowCount) += 1;
			}

			for (i = 0; i < numscalars; i++)
			{
				*outPtr = (*inPtr)/convertion_ratio;
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
		default:
			std::string s = "Unknown Compounding operator detected, value " + std::to_string(compoundingMode) + ". Leaving value as-is.";
			utilities::cout << s;
			break;
		}
		return 1;
	}
	return 0;
}

template<typename input_pixel_type,typename output_pixel_type,double convertion_ration>
struct PasteSliceIntoVolumeInsertSliceParams
{
	// information on the volume
	itk::Image<input_pixel_type,3>::Pointer outData;            // the output volume
	output_pixel_type* outPtr;                     // scalar pointer to the output volume over the output extent
	unsigned short* accPtr;           // scalar pointer to the accumulation buffer over the output extent
	itk::Image<output_pixel_type,3>::Pointer inData;             // input slice
	input_pixel_type* inPtr;                      // scalar pointer to the input volume over the input slice extent
	int* inExt;                       // array size 6, input slice extent (could have been split for threading)
	unsigned int* accOverflowCount;   // the number of voxels that may have error due to accumulation overflow

	// transform matrix for images -> volume
	Eigen::Matrix4d matrix;

	// details specified by the user RE: how the voxels should be computed
	Interpolation interpolationMode;   // linear or nearest neighbor
	Compounding compoundingMode;

	// parameters for clipping
	double* clipRectangleOrigin; // array size 2
	double* clipRectangleSize; // array size 2

	double pixelRejectionThreshold;
	int image_number;
};

template<typename input_pixel_type,typename output_pixel_type,double convertion_ration>
void UnoptimizedInsertSlice(PasteSliceIntoVolumeInsertSliceParams<input_pixel_type,output_pixel_type,convertion_ration>* insertionParams){

};

}
}
}

#endif