#include "imageprocessing/VolumeAlgorithms.h"
#include "utils/Logger.h"

namespace curan {
namespace image {
namespace reconstruction{

bool DeleteVolume()
{
	return true;
}

bool GetObliqueSlice(Volume& in_vol, InternalImageType::Pointer& out_vol, itk::Matrix<double, 4, 4> transformation)
{
	using LinearInterpolatorType = itk::LinearInterpolateImageFunction<InternalImageType, double>;
	typename LinearInterpolatorType::Pointer interpolator = LinearInterpolatorType::New();
	auto inputRegion = in_vol.volume->GetLargestPossibleRegion();
	auto size = inputRegion.GetSize();
	size[2] = 1;

	SlicerType::Pointer filter = SlicerType::New();
	filter->SetInterpolator(interpolator);
	filter->SetSize(size);
	filter->SetDefaultPixelValue(0);
	filter->SetInput(in_vol.volume);
	return true;
}

bool GetPerpendicularSlice(Volume& in_vol, InternalImageType::Pointer& out_vol, uint8_t direction, uint32_t slice_number)
{
	using FilterType = itk::RegionOfInterestImageFilter<InternalImageType, InternalImageType>;
	FilterType::Pointer filter = FilterType::New();
	filter->SetInput(in_vol.volume);
	// set up the extraction region [one slice]
	InternalImageType::RegionType inputRegion = in_vol.volume->GetLargestPossibleRegion();
	InternalImageType::SizeType   size = inputRegion.GetSize();
	size[(int)direction] = 1;

	InternalImageType::IndexType start = inputRegion.GetIndex();
	start[(int)direction] = slice_number;

	InternalImageType::RegionType desiredRegion;
	desiredRegion.SetSize(size);
	desiredRegion.SetIndex(start);

	filter->SetRegionOfInterest(desiredRegion);

	try {
		filter->Update();
	}
	catch (const itk::ExceptionObject& e) {
		utilities::cout << e.what();
		return false;
	}

	itk::SmartPointer<InternalImageType> output_image = filter->GetOutput();
	output_image->DisconnectPipeline();
	InternalImageType::RegionType inputRegion2 = output_image->GetBufferedRegion();
	InternalImageType::SizeType   size2 = inputRegion2.GetSize();
	out_vol = output_image;
	return true;
}

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

int TrilinearInterpolation(const Eigen::Vector4d point,
	char_pixel_type* inPtr,
	char_pixel_type* outPtr,
	unsigned short* accPtr,
	int numscalars,
	VolumeReconstructor::Compounding compoundingMode,
	int outExt[6],
	uint64_t outInc[3],
	unsigned int* accOverflowCount)
{
	// Determine if the output is a floating point or integer type. If floating point type then we don't round
	// the interpolated value.
	bool roundOutput = true; // assume integer output by default
	char_pixel_type floatValueInOutputType = 0.3;
	if (floatValueInOutputType > 0)
	{
		// output is a floating point number
		roundOutput = false;
	}

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
		char_pixel_type* inPtrTmp, * outPtrTmp;

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
				case VolumeReconstructor::MAXIMUM_COMPOUNDING_MODE:
				{
					const double minWeight(0.125); // If a pixel is right in the middle of the eight surrounding voxels
					// (trilinear weight = 0.125 for each), then it the compounding operator
					// should be applied for each. Else, it should only be considered
					// for the other nearest voxels.
					if (fdx[j] >= minWeight && *inPtrTmp > *outPtrTmp)
					{
						*outPtrTmp = (*inPtrTmp);
						f = fdx[j];
						a = f * ACCUMULATION_MULTIPLIER;;
					}
					break;
				}
				case VolumeReconstructor::LATEST_COMPOUNDING_MODE:
				{
					const double minWeight(0.125); // If a pixel is right in the middle of the eight surrounding voxels
					// (trilinear weight = 0.125 for each), then it the compounding operator
					// should be applied for each. Else, it should only be considered
					// for the other nearest voxels.
					if (fdx[j] >= minWeight)
					{
						*outPtrTmp = (*inPtrTmp);
						f = fdx[j];
						a = f * ACCUMULATION_MULTIPLIER;;
					}
					break;
				}
				case VolumeReconstructor::MEAN_COMPOUNDING_MODE:
					f = fdx[j];
					r = double((*accPtrTmp) / (double)ACCUMULATION_MULTIPLIER); // added division by double, since this always returned 0 otherwise
					a = f + r;
					if (roundOutput)
					{
						*outPtrTmp = std::round((f * (*inPtrTmp) + r * (*outPtrTmp)) / a);
					}
					else
					{
						*outPtrTmp = (f * (*inPtrTmp) + r * (*outPtrTmp)) / a;
					}
					a *= ACCUMULATION_MULTIPLIER; // needs to be done for proper conversion to unsigned short for accumulation buffer
					break;
				default:
					std::string s = "Unknown Compounding operator detected, value " + std::to_string(compoundingMode) + ". Leaving value as-is.";
					utilities::cout << s;
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

int NearestNeighborInterpolation(const Eigen::Vector4d point,
	char_pixel_type* inPtr,
	char_pixel_type* outPtr,
	unsigned short* accPtr,
	int numscalars,
	VolumeReconstructor::Compounding compoundingMode,
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

	/*

	const size_t string_maximum_size = (12 + 1) * 3 + 1;
	char str[string_maximum_size];

	int number_writen = 0;
	number_writen += sprintf(str + number_writen, "%d", outIdX);
	str[number_writen] = ',';
	number_writen += 1;
	number_writen += sprintf(str + number_writen, "%d", outIdY);
	str[number_writen] = ',';
	number_writen += 1;
	number_writen += sprintf(str + number_writen, "%d", outIdZ);
	curan::utils::console->info(str);
	*/
	// fancy way of checking bounds
	if ((outIdX | (outExt[1] - outExt[0] - outIdX) |
		outIdY | (outExt[3] - outExt[2] - outIdY) |
		outIdZ | (outExt[5] - outExt[4] - outIdZ)) >= 0)
	{
		int inc = outIdX * outInc[0] + outIdY * outInc[1] + outIdZ * outInc[2];
		outPtr += inc;
		switch (compoundingMode)
		{
		case (VolumeReconstructor::MAXIMUM_COMPOUNDING_MODE):
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
					*outPtr = *inPtr;
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
		case (VolumeReconstructor::MEAN_COMPOUNDING_MODE):
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
					*outPtr = ((*inPtr++) * ACCUMULATION_MULTIPLIER + (*outPtr) * (*accPtr)) / newa;
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
		case (VolumeReconstructor::LATEST_COMPOUNDING_MODE):
		{
			accPtr += inc / outInc[0];

			int newa = *accPtr + ACCUMULATION_MULTIPLIER;
			if (newa > ACCUMULATION_THRESHOLD)
			{
				(*accOverflowCount) += 1;
			}

			for (i = 0; i < numscalars; i++)
			{
				*outPtr = *inPtr;
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

bool ApplyNearestNeighbor(char_pixel_type* inputData,
	unsigned short* accData,
	uint64_t* inputOffsets,
	uint64_t* bounds,
	uint64_t* wholeExtent,
	uint64_t* thisPixel,
	char_pixel_type& returnVal,
	const VolumeReconstructor::KernelDescriptor* descrip)
{
	double sumIntensities(0); // unsigned long because these rise in value quickly
	int sumAccumulator(0);
	int maxRange((descrip->size - 1) / 2);
	bool exit(false);

	for (int range = 1; range <= maxRange && !exit; range++) {
		int minX = thisPixel[0] - range;
		int minY = thisPixel[1] - range;
		int minZ = thisPixel[2] - range;
		int maxX = thisPixel[0] + range;
		int maxY = thisPixel[1] + range;
		int maxZ = thisPixel[2] + range;
		for (int x = minX; x <= maxX; x++)
		{
			for (int y = minY; y <= maxY; y++)
			{
				for (int z = minZ; z <= maxZ; z++)
				{
					if (x <= wholeExtent[1] && x >= wholeExtent[0] &&
						y <= wholeExtent[3] && y >= wholeExtent[2] &&
						z <= wholeExtent[5] && z >= wholeExtent[4]) // check bounds
					{
						int accIndex = inputOffsets[0] * x + inputOffsets[1] * y + inputOffsets[2] * z;
						if (accData[accIndex]) { // if the accumulation buffer for the voxel is non-zero
							sumIntensities += inputData[accIndex];
							sumAccumulator++;
							exit = true;
						}
					} // end boundary check
				} // end z loop
			} // end y loop
		} // end x loop
	} // end range loop

	if (sumAccumulator == 0) { // no voxels set in the area
		returnVal = (char_pixel_type)0;
		return false;
	}

	if ((double)sumAccumulator / (descrip->size * descrip->size * descrip->size) > descrip->minRatio)
	{
		returnVal = (char_pixel_type)(sumIntensities / sumAccumulator); // set it if and only if the min ratio is met
		return true;
	}

	// else failure
	returnVal = (char_pixel_type)0;
	return false;

};

bool ApplyDistanceWeightInverse(char_pixel_type* inputData,
	unsigned short* accData,
	uint64_t* inputOffsets,
	uint64_t* bounds,
	uint64_t* wholeExtent,
	uint64_t* thisPixel,
	char_pixel_type& returnVal,
	const VolumeReconstructor::KernelDescriptor* descrip)
{
	// set the x, y, and z range
	int range = (descrip->size - 1) / 2; // so with N = 3, our range is x-1 through x+1, and so on
	int minX = thisPixel[0] - range;
	int minY = thisPixel[1] - range;
	int minZ = thisPixel[2] - range;
	int maxX = thisPixel[0] + range;
	int maxY = thisPixel[1] + range;
	int maxZ = thisPixel[2] + range;

	double sumIntensities(0); // unsigned long because these rise in value quickly
	double sumAccumulator(0);
	unsigned short currentAccumulation(0);
	int numKnownVoxels(0);

	for (int x = minX; x <= maxX; x++)
	{
		for (int y = minY; y <= maxY; y++)
		{
			for (int z = minZ; z <= maxZ; z++)
			{
				if (x <= wholeExtent[1] && x >= wholeExtent[0] &&
					y <= wholeExtent[3] && y >= wholeExtent[2] &&
					z <= wholeExtent[5] && z >= wholeExtent[4]) // check bounds
				{
					int accIndex = inputOffsets[0] * x + inputOffsets[1] * y + inputOffsets[2] * z;
					currentAccumulation = accData[accIndex];
					if (currentAccumulation) { // if the accumulation buffer for the voxel is non-zero
						int kerIndex = descrip->size * descrip->size * (z - minZ) + descrip->size * (y - minY) + (x - minX);
						double weight = descrip->kernel[kerIndex];
						sumIntensities += inputData[accIndex] * weight;
						sumAccumulator += weight;
						numKnownVoxels++;
					}
				} // end boundary check
			} // end z loop
		} // end y loop
	} // end x loop

	if (sumAccumulator == 0) { // no voxels set in the area
		returnVal = (char_pixel_type)0;
		return false;
	}

	if ((double)numKnownVoxels / (descrip->size * descrip->size * descrip->size) > descrip->minRatio)
	{
		returnVal = (char_pixel_type)(sumIntensities / sumAccumulator); // set it if and only if the min ratio is met
		return true;
	}

	// else failure
	returnVal = (char_pixel_type)0;
	return false;
};

bool ApplyGaussian(char_pixel_type* inputData,
	unsigned short* accData,
	uint64_t* inputOffsets,
	uint64_t* bounds,
	uint64_t* wholeExtent,
	uint64_t* thisPixel,
	char_pixel_type& returnVal,
	const VolumeReconstructor::KernelDescriptor* descrip)
{
	// set the x, y, and z range
	int range = (descrip->size - 1) / 2; // so with N = 3, our range is x-1 through x+1, and so on
	int minX = thisPixel[0] - range;
	int minY = thisPixel[1] - range;
	int minZ = thisPixel[2] - range;
	int maxX = thisPixel[0] + range;
	int maxY = thisPixel[1] + range;
	int maxZ = thisPixel[2] + range;

	double sumIntensities(0); // unsigned long because these rise in value quickly
	double sumAccumulator(0);
	unsigned short currentAccumulation(0);
	int numKnownVoxels(0);

	for (int x = minX; x <= maxX; x++)
	{
		for (int y = minY; y <= maxY; y++)
		{
			for (int z = minZ; z <= maxZ; z++)
			{
				if (x <= wholeExtent[1] && x >= wholeExtent[0] &&
					y <= wholeExtent[3] && y >= wholeExtent[2] &&
					z <= wholeExtent[5] && z >= wholeExtent[4]) // check bounds
				{
					int accIndex = inputOffsets[0] * x + inputOffsets[1] * y + inputOffsets[2] * z;
					currentAccumulation = accData[accIndex];
					if (currentAccumulation) { // if the accumulation buffer for the voxel is non-zero
						int kerIndex = descrip->size * descrip->size * (z - minZ) + descrip->size * (y - minY) + (x - minX);
						double weight = descrip->kernel[kerIndex];
						sumIntensities += inputData[accIndex] * weight;
						sumAccumulator += weight;
						numKnownVoxels++;
					}
				} // end boundary check
			} // end z loop
		} // end y loop
	} // end x loop

	if (sumAccumulator == 0) { // no voxels set in the area
		returnVal = (char_pixel_type)0;
		return false;
	}

	if ((double)numKnownVoxels / (descrip->size * descrip->size * descrip->size) > descrip->minRatio)
	{
		returnVal = (char_pixel_type)(sumIntensities / sumAccumulator); // set it if and only if the min ratio is met
		return true;
	}

	// else failure
	returnVal = (char_pixel_type)0;
	return false;
};

bool ApplyGaussianAccumulation(char_pixel_type* inputData,
	unsigned short* accData,
	uint64_t* inputOffsets,
	uint64_t* bounds,
	uint64_t* wholeExtent,
	uint64_t* thisPixel,
	char_pixel_type& returnVal,
	const VolumeReconstructor::KernelDescriptor* descrip)
{
	// set the x, y, and z range
	int range = (descrip->size - 1) / 2; // so with N = 3, our range is x-1 through x+1, and so on
	int minX = thisPixel[0] - range;
	int minY = thisPixel[1] - range;
	int minZ = thisPixel[2] - range;
	int maxX = thisPixel[0] + range;
	int maxY = thisPixel[1] + range;
	int maxZ = thisPixel[2] + range;

	double sumIntensities(0); // unsigned long because these rise in value quickly
	double sumAccumulator(0);
	unsigned short currentAccumulation(0);
	int numKnownVoxels(0);

	for (int x = minX; x <= maxX; x++)
	{
		for (int y = minY; y <= maxY; y++)
		{
			for (int z = minZ; z <= maxZ; z++)
			{
				if (x <= wholeExtent[1] && x >= wholeExtent[0] &&
					y <= wholeExtent[3] && y >= wholeExtent[2] &&
					z <= wholeExtent[5] && z >= wholeExtent[4]) // check bounds
				{
					int accIndex = inputOffsets[0] * x + inputOffsets[1] * y + inputOffsets[2] * z;
					currentAccumulation = accData[accIndex];
					if (currentAccumulation) { // if the accumulation buffer for the voxel is non-zero
						int kerIndex = descrip->size * descrip->size * (z - minZ) + descrip->size * (y - minY) + (x - minX);
						double weight = currentAccumulation * descrip->kernel[kerIndex];
						sumIntensities += inputData[accIndex] * weight;
						sumAccumulator += weight;
						numKnownVoxels++;
					}
				} // end boundary check
			} // end z loop
		} // end y loop
	} // end x loop

	if (sumAccumulator == 0) { // no voxels set in the area
		returnVal = (char_pixel_type)0;
		return false;
	}

	if ((double)numKnownVoxels / (descrip->size * descrip->size * descrip->size) > descrip->minRatio)
	{
		returnVal = (char_pixel_type)(sumIntensities / sumAccumulator); // set it if and only if the min ratio is met
		return true;
	}

	// else failure
	returnVal = (char_pixel_type)0;
	return false;
};

bool ApplySticks(char_pixel_type* inputData,
	unsigned short* accData,
	uint64_t* inputOffsets,
	uint64_t* bounds,
	uint64_t* wholeExtent,
	uint64_t* thisPixel,
	char_pixel_type& returnVal,
	const VolumeReconstructor::KernelDescriptor* descrip)
{
	// extract coordinates
	int x(thisPixel[0]), y(thisPixel[1]), z(thisPixel[2]); // coordinates of the hole voxel
	int xtemp, ytemp, ztemp; // store the voxel along the current stick
	bool valid; // set to true when we've hit a filled voxel
	int fwdTrav, rvsTrav; // store the number of voxels that have been searched
	char_pixel_type fwdVal, rvsVal; // store the values at each end of the stick
	char_pixel_type* values = new char_pixel_type[descrip->numSticksInList];
	double* weights = new double[descrip->numSticksInList];
	// try each stick direction
	for (int i = 0; i < descrip->numSticksInList; i++) {

		int baseStickIndex = i * 3; // 3 coordinates per stick, one for each dimension

		// evaluate forward direction to nearest filled voxel
		xtemp = x; ytemp = y; ztemp = z;
		valid = false;
		for (int j = 1; j + 1 <= descrip->stickLengthLimit; j++) {
			// traverse in forward direction
			xtemp = xtemp + descrip->sticksList[baseStickIndex];
			ytemp = ytemp + descrip->sticksList[baseStickIndex + 1];
			ztemp = ztemp + descrip->sticksList[baseStickIndex + 2];
			// check boundaries
			if (xtemp > wholeExtent[1] || xtemp < wholeExtent[0] ||
				ytemp > wholeExtent[3] || ytemp < wholeExtent[2] ||
				ztemp > wholeExtent[5] || ztemp < wholeExtent[4]) // check bounds
					break;
			int accIndex = inputOffsets[0] * xtemp + inputOffsets[1] * ytemp + inputOffsets[2] * ztemp;
			if (accData[accIndex] != 0) { // this is a filled voxel
				fwdTrav = j;
				fwdVal = inputData[accIndex];
				valid = true;
					break;
			}
		} // end searching fwd direction

		// only do reverse direction if we found something forward
		if (!valid) {
			weights[i] = 0.0; // do this to say that this is a bad stick
			continue; // try next stick
		}

		// evaluate reverse direction to nearest filled voxel
		xtemp = x; ytemp = y; ztemp = z;
		valid = false;
		for (int j = 1; j + fwdTrav + 1 <= descrip->stickLengthLimit; j++) {
			// traverse in reverse direction
			xtemp = xtemp - descrip->sticksList[baseStickIndex];
			ytemp = ytemp - descrip->sticksList[baseStickIndex + 1];
			ztemp = ztemp - descrip->sticksList[baseStickIndex + 2];
			// check boundaries
			if (xtemp > wholeExtent[1] || xtemp < wholeExtent[0] ||
				ytemp > wholeExtent[3] || ytemp < wholeExtent[2] ||
				ztemp > wholeExtent[5] || ztemp < wholeExtent[4]) // check bounds
					break;
			int accIndex = inputOffsets[0] * xtemp + inputOffsets[1] * ytemp + inputOffsets[2] * ztemp;
			if (accData[accIndex] != 0) { // this is a filled voxel
				rvsTrav = j;
				rvsVal = inputData[accIndex];
				valid = true;
				break;
			}
		} // end searching rvs direction

		// only calculate a score and a value if we found something in both directions
		if (!valid) {
			weights[i] = 0.0; // do this to say that this is a bad stick
			continue; // try next stick
		}

		// evaluate score and direction
		double totalDistance = (fwdTrav + rvsTrav + 1);
		double weightFwd = (rvsTrav + 1) / totalDistance;
		double weightRvs = 1.0 - weightFwd;
		double realDistance = totalDistance * sqrt((double)(descrip->sticksList[baseStickIndex] * descrip->sticksList[baseStickIndex] +
			descrip->sticksList[baseStickIndex + 1] * descrip->sticksList[baseStickIndex + 1] +
			descrip->sticksList[baseStickIndex + 2] * descrip->sticksList[baseStickIndex + 2]));
		weights[i] = 1.0 / realDistance;
		values[i] = weightRvs * rvsVal + weightFwd * fwdVal;
	}

	// determine the highest score, and assign the corresponding value to the pixel
	int numSticksUsed(0);
	double sumWeightedValues(0.0);
	double sumWeights(0.0);

	// iterate through sticks to find the maximum score, use that stick in the calculation, then set the score for it to 0, and repeat
	while (numSticksUsed < descrip->numSticksToUse) {

		// determine highest score among remaining sticks
		double maxWeight(0.0);
		for (int i = 0; i < descrip->numSticksInList; i++) {
			if (weights[i] > maxWeight) {
				maxWeight = weights[i];
			}
		}

		if (maxWeight == 0) { // indicates all sticks were bad
			break;
		}

		// for all sticks with this weight, use them in the result
		for (int i = 0; i < descrip->numSticksInList; i++) {
			if (weights[i] == maxWeight) {
				sumWeightedValues += (values[i] * weights[i]);
				sumWeights += weights[i];
				numSticksUsed++;
				weights[i] = 0.0;
			}
		}

	}

	delete[] weights;
	delete[] values;

	if (sumWeights != 0) {
		returnVal = (char_pixel_type)(sumWeightedValues / sumWeights);
		return true; // at least one stick was good, = success
	}

	// else sumWeights = 0 means all sticks were bad
	returnVal = (char_pixel_type)0;
	return false;

}

bool PixelRejectionEnabled(double threshold)
{
	return threshold > PIXEL_REJECTION_DISABLED + DBL_MIN * 200;
};

void UnoptimizedInsertSlice(PasteSliceIntoVolumeInsertSliceParams* insertionParams)
{
	// information on the volume
	InternalImageType::Pointer outData = insertionParams->outData;
	char_pixel_type* outPtr = outData->GetBufferPointer();
	unsigned short* accPtr = insertionParams->accPtr;
	InternalImageType::Pointer inData = insertionParams->inData;
	char_pixel_type* inPtr = inData->GetBufferPointer();
	int* inExt = insertionParams->inExt;
	unsigned int* accOverflowCount = insertionParams->accOverflowCount;

	// details specified by the user RE: how the voxels should be computed
	VolumeReconstructor::Interpolation interpolationMode = insertionParams->interpolationMode;   // linear or nearest neighbor
	VolumeReconstructor::Compounding compoundingMode = insertionParams->compoundingMode;         // weighted average or maximum

	// parameters for clipping
	double* clipRectangleOrigin = insertionParams->clipRectangleOrigin; // array size 2
	double* clipRectangleSize = insertionParams->clipRectangleSize; // array size 2

	// slice spacing and origin
	double inSpacing[3];
	auto spacing = inData->GetSpacing();
	inSpacing[0] = spacing[0];
	inSpacing[1] = spacing[1];
	inSpacing[2] = spacing[2];

	double inOrigin[3];
	auto origin = inData->GetOrigin();
	inOrigin[0] = origin[0];
	inOrigin[1] = origin[1];
	inOrigin[2] = origin[2];

	// get the clip rectangle as an extent
	int clipExt[6];
	GetClipExtent(clipExt, inOrigin, inSpacing, inExt, clipRectangleOrigin, clipRectangleSize);

	// find maximum output range = output extent
	int outExt[6];
	double outSpacing[3];
	auto outspacing = outData->GetSpacing();
	outSpacing[0] = outspacing[0];
	outSpacing[1] = outspacing[1];
	outSpacing[2] = outspacing[2];

	auto outdata_ROI = outData->GetLargestPossibleRegion();
	auto size_out = outdata_ROI.GetSize();
	auto start_out = outdata_ROI.GetIndex();

	outExt[0] = start_out[0];
	outExt[1] = size_out[0] - 1;

	outExt[2] = start_out[1];
	outExt[3] = size_out[1] - 1;

	outExt[4] = start_out[2];
	outExt[5] = size_out[2] - 1;

	// Get increments to march through data - ex move from the end of one x scanline of data to the
	// start of the next line
	uint64_t outInc[3] = { 0,0,0 };

	uint64_t  incr = VolumeReconstructor::INPUT_COMPONENTS;
	for (int i = 0; i < 3; ++i) {
		outInc[i] = incr;
		incr *= (outExt[i * 2 + 1] - outExt[i * 2] + 1);
	}

	uint64_t inIncX = 0, inIncY = 0, inIncZ = 0;

	auto indata_ROI = inData->GetLargestPossibleRegion();
	auto size_in = indata_ROI.GetSize();
	auto start_in = indata_ROI.GetIndex();


	// Get increments to march through data - ex move from the end of one x scanline of data to the
	// start of the next line
	uint64_t inInc[3] = { 0,0,0 };

	incr = VolumeReconstructor::INPUT_COMPONENTS;
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

	int numscalars = VolumeReconstructor::INPUT_COMPONENTS;

	// Set interpolation method - nearest neighbor or trilinear
	int (*interpolate)(const Eigen::Vector4d, char_pixel_type*, char_pixel_type*, unsigned short*, int, VolumeReconstructor::Compounding, int a[6], uint64_t b[3], unsigned int*) = NULL;     // pointer to the nearest neighbor or trilinear interpolation function
	switch (interpolationMode)
	{
	case VolumeReconstructor::NEAREST_NEIGHBOR_INTERPOLATION:
		interpolate = &NearestNeighborInterpolation;
		break;
	case VolumeReconstructor::LINEAR_INTERPOLATION:
		interpolate = &TrilinearInterpolation;
		break;
	default:
	{
		std::string s = "Unknown interpolation mode: " + std::to_string(interpolationMode);
		utilities::cout << s;
		return;
	}
	}

	// Loop through  slice pixels in the input extent and put them into the output volume
	// the resulting point in the output volume (outPoint) from a point in the input slice
	// (inpoint)
	Eigen::Vector4d inPoint;
	Eigen::Vector4d outPoint;
	inPoint[3] = 1;

	for (int idZ = inExt[4]; idZ <= inExt[5]; idZ++, inPtr += inIncZ)
	{
		for (int idY = inExt[2]; idY <= inExt[3]; idY++, inPtr += inIncY)
		{
			for (int idX = inExt[0]; idX <= inExt[1]; idX++, inPtr += numscalars)
			{
				// check if we are within the current clip extent
				if (idX < clipExt[0] || idX > clipExt[1] || idY < clipExt[2] || idY > clipExt[3])
				{
					// outside the clipping rectangle
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

				/*
				const size_t string_maximum_size = (12 + 1) * 4 + 1;
				char str[string_maximum_size];

				int number_writen = 0;
				number_writen += sprintf(str + number_writen, "%d", insertionParams->image_number);
				str[number_writen] = ',';
				number_writen += 1;
				number_writen += sprintf(str + number_writen, "%d", idX);
				str[number_writen] = ',';
				number_writen += 1;
				number_writen += sprintf(str + number_writen, "%d", idY);
				str[number_writen] = ',';
				number_writen += 1;
				number_writen += sprintf(str + number_writen, "%d", idZ);

				curan::utils::console->info(str);
				*/
				// interpolation functions return 1 if the interpolation was successful, 0 otherwise
				interpolate(outPoint, inPtr, outPtr, accPtr, numscalars, compoundingMode, outExt, outInc, accOverflowCount);
			}
		}
	}
}

}
}
}