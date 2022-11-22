#include "PkLibrary.h"

namespace curan {
	namespace image {
		Result DeleteVolume()
		{
			return Result::PK_SUCCESS;
		}

		Result GetObliqueSlice(Volume& in_vol, InternalImageType::Pointer& out_vol, itk::Matrix<double, 4, 4> transformation)
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
			return Result::PK_VOLUME_NOT_PRESENT;
		}

		Result GetPerpendicularSlice(Volume& in_vol, InternalImageType::Pointer& out_vol, uint8_t direction, uint32_t slice_number)
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
				curan::utils::console->info(e.what());
				return Result::PK_VOLUME_NOT_PRESENT;
			}

			itk::SmartPointer<InternalImageType> output_image = filter->GetOutput();
			output_image->DisconnectPipeline();
			InternalImageType::RegionType inputRegion2 = output_image->GetBufferedRegion();
			InternalImageType::SizeType   size2 = inputRegion2.GetSize();
			out_vol = output_image;
			return Result::PK_SUCCESS;
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
			CharPixelType* inPtr,
			CharPixelType* outPtr,
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
			CharPixelType floatValueInOutputType = 0.3;
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
				CharPixelType* inPtrTmp, * outPtrTmp;

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
							curan::utils::console->info(s);
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
			CharPixelType* inPtr,
			CharPixelType* outPtr,
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
					}
					else     // overflow, use recursive filtering with 255/256 and 1/256 as the weights, since 255 voxels have been inserted so far
					{
						// TODO: Should do this for all the scalars, and accumulation?
						*outPtr = (CharPixelType)(fraction1_256 * (*inPtr++) + fraction255_256 * (*outPtr));
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
					curan::utils::console->info(s);
					break;
				}
				return 1;
			}
			return 0;
		}

		bool ApplyNearestNeighbor(CharPixelType* inputData,
			unsigned short* accData,
			uint64_t* inputOffsets,
			uint64_t* bounds,
			uint64_t* wholeExtent,
			uint64_t* thisPixel,
			CharPixelType& returnVal,
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
				returnVal = (CharPixelType)0;
				return false;
			}

			if ((double)sumAccumulator / (descrip->size * descrip->size * descrip->size) > descrip->minRatio)
			{
				returnVal = (CharPixelType)(sumIntensities / sumAccumulator); // set it if and only if the min ratio is met
				return true;
			}

			// else failure
			returnVal = (CharPixelType)0;
			return false;

		};

		bool ApplyDistanceWeightInverse(CharPixelType* inputData,
			unsigned short* accData,
			uint64_t* inputOffsets,
			uint64_t* bounds,
			uint64_t* wholeExtent,
			uint64_t* thisPixel,
			CharPixelType& returnVal,
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
				returnVal = (CharPixelType)0;
				return false;
			}

			if ((double)numKnownVoxels / (descrip->size * descrip->size * descrip->size) > descrip->minRatio)
			{
				returnVal = (CharPixelType)(sumIntensities / sumAccumulator); // set it if and only if the min ratio is met
				return true;
			}

			// else failure
			returnVal = (CharPixelType)0;
			return false;
		};

		bool ApplyGaussian(CharPixelType* inputData,
			unsigned short* accData,
			uint64_t* inputOffsets,
			uint64_t* bounds,
			uint64_t* wholeExtent,
			uint64_t* thisPixel,
			CharPixelType& returnVal,
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
				returnVal = (CharPixelType)0;
				return false;
			}

			if ((double)numKnownVoxels / (descrip->size * descrip->size * descrip->size) > descrip->minRatio)
			{
				returnVal = (CharPixelType)(sumIntensities / sumAccumulator); // set it if and only if the min ratio is met
				return true;
			}

			// else failure
			returnVal = (CharPixelType)0;
			return false;
		};

		bool ApplyGaussianAccumulation(CharPixelType* inputData,
			unsigned short* accData,
			uint64_t* inputOffsets,
			uint64_t* bounds,
			uint64_t* wholeExtent,
			uint64_t* thisPixel,
			CharPixelType& returnVal,
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
				returnVal = (CharPixelType)0;
				return false;
			}

			if ((double)numKnownVoxels / (descrip->size * descrip->size * descrip->size) > descrip->minRatio)
			{
				returnVal = (CharPixelType)(sumIntensities / sumAccumulator); // set it if and only if the min ratio is met
				return true;
			}

			// else failure
			returnVal = (CharPixelType)0;
			return false;
		};

		bool ApplySticks(CharPixelType* inputData,
			unsigned short* accData,
			uint64_t* inputOffsets,
			uint64_t* bounds,
			uint64_t* wholeExtent,
			uint64_t* thisPixel,
			CharPixelType& returnVal,
			const VolumeReconstructor::KernelDescriptor* descrip)
		{
			// extract coordinates
			int x(thisPixel[0]), y(thisPixel[1]), z(thisPixel[2]); // coordinates of the hole voxel
			int xtemp, ytemp, ztemp; // store the voxel along the current stick
			bool valid; // set to true when we've hit a filled voxel
			int fwdTrav, rvsTrav; // store the number of voxels that have been searched
			CharPixelType fwdVal, rvsVal; // store the values at each end of the stick

			CharPixelType* values = new CharPixelType[descrip->numSticksInList];
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
				returnVal = (CharPixelType)(sumWeightedValues / sumWeights);
				return true; // at least one stick was good, = success
			}

			// else sumWeights = 0 means all sticks were bad
			returnVal = (CharPixelType)0;
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
			CharPixelType* outPtr = outData->GetBufferPointer();
			unsigned short* accPtr = insertionParams->accPtr;
			InternalImageType::Pointer inData = insertionParams->inData;
			CharPixelType* inPtr = inData->GetBufferPointer();
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
			int (*interpolate)(const Eigen::Vector4d, CharPixelType*, CharPixelType*, unsigned short*, int, VolumeReconstructor::Compounding, int a[6], uint64_t b[3], unsigned int*) = NULL;     // pointer to the nearest neighbor or trilinear interpolation function
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
				curan::utils::console->info(s);
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


		VolumeReconstructor::VolumeReconstructor()
		{
			if (out_volume.IsNull())
			{
				out_volume = InternalImageType::New();
			}

			if (acummulation_buffer.IsNull())
			{
				acummulation_buffer = itk::Image<ShortPixelType, Dimension3D>::New();
			}

			output_spacing[0] = 1.0;
			output_spacing[1] = 1.0;
			output_spacing[2] = 1.0;

		}

		VolumeReconstructor::~VolumeReconstructor()
		{
		}

		void VolumeReconstructor::Update()
		{
			std::vector<gte::Vector3<double>> vertices;

			// We multiply by four because each 
			// image has four courners and we 
			// add eight because we have the 
			// current eight corners in memory 
			// which represent the minimum bounding 
			// box containing all the frames 
			// already added to memory
			vertices.resize(frame_data.size() * 4 + 8);

			int increment = 0;
			for (auto img : frame_data)
			{
				auto size = img->GetLargestPossibleRegion().GetSize();
				int height = size[1];
				int width = size[0];

				OutputType::PointType origin_position;
				OutputType::IndexType origin_pixel = { 0,0,0 };
				img->TransformIndexToPhysicalPoint(origin_pixel, origin_position);

				vertices[increment] = gte::Vector3<double>({ origin_position[0], origin_position[1], origin_position[2] });

				OutputType::IndexType origin_along_width = { width - 1,0,0 };
				OutputType::PointType origin_along_width_position;
				img->TransformIndexToPhysicalPoint(origin_along_width, origin_along_width_position);

				vertices[increment + 1] = gte::Vector3<double>({ origin_along_width_position[0], origin_along_width_position[1], origin_along_width_position[2] });

				OutputType::IndexType origin_along_width_and_height = { width - 1,height - 1,0 };
				OutputType::PointType origin_along_width_and_height_position;
				img->TransformIndexToPhysicalPoint(origin_along_width_and_height, origin_along_width_and_height_position);

				vertices[increment + 2] = gte::Vector3<double>({ origin_along_width_and_height_position[0], origin_along_width_and_height_position[1], origin_along_width_and_height_position[2] });

				OutputType::IndexType origin_along_height = { 0,height - 1,0 };
				OutputType::PointType origin_along_height_position;
				img->TransformIndexToPhysicalPoint(origin_along_height, origin_along_height_position);

				vertices[increment + 3] = gte::Vector3<double>({ origin_along_height_position[0], origin_along_height_position[1], origin_along_height_position[2] });

				increment += 4;
			};

			// if the bounding box is still unitialized 
			// then we need to run the algorithm without 
			// the vertices stored in current_corners
			if (!volumes_initiated)
			{
				gte::MinimumVolumeBox3<double, true> bounding_box(0);

				double volume = 0.0;
				bounding_box(frame_data.size() * 4, vertices.data(), 4, volumetric_bounding_box, volume);
			}
			else
			{
				std::array<gte::Vector3<double>, 8> current_corners;
				current_corners[0] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
				current_corners[1] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
				current_corners[2] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
				current_corners[3] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
				current_corners[4] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
				current_corners[5] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
				current_corners[6] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
				current_corners[7] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];

				vertices.insert(vertices.begin() + frame_data.size() * 4, std::begin(current_corners), std::begin(current_corners));
				gte::MinimumVolumeBox3<double, true> bounding_box(0);

				double volume = 0.0;
				bounding_box(vertices.size(), vertices.data(), 4, volumetric_bounding_box, volume);
			}

			//since we might have enlarged the size of the output volume with the new pixels added we need to update the current volume.
			UpdateInternalBuffers();

			gte::Vector<3, double> output_origin = volumetric_bounding_box.center
				- volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0]
				- volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1]
				- volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];

			Eigen::Matrix4d ref_to_output_origin;
			ref_to_output_origin(0, 0) = volumetric_bounding_box.axis[0][0];
			ref_to_output_origin(1, 0) = volumetric_bounding_box.axis[0][1];
			ref_to_output_origin(2, 0) = volumetric_bounding_box.axis[0][2];
			ref_to_output_origin(3, 0) = 0.0;

			ref_to_output_origin(0, 1) = volumetric_bounding_box.axis[1][0];
			ref_to_output_origin(1, 1) = volumetric_bounding_box.axis[1][1];
			ref_to_output_origin(2, 1) = volumetric_bounding_box.axis[1][2];
			ref_to_output_origin(3, 1) = 0.0;

			ref_to_output_origin(0, 2) = volumetric_bounding_box.axis[2][0];
			ref_to_output_origin(1, 2) = volumetric_bounding_box.axis[2][1];
			ref_to_output_origin(2, 2) = volumetric_bounding_box.axis[2][2];
			ref_to_output_origin(3, 2) = 0.0;

			ref_to_output_origin(0, 3) = output_origin[0];
			ref_to_output_origin(1, 3) = output_origin[1];
			ref_to_output_origin(2, 3) = output_origin[2];
			ref_to_output_origin(3, 3) = 1.0;

			Eigen::Matrix4d output_to_ref = ref_to_output_origin.inverse();

			unsigned int accOverflow = 20;
			int inputFrameExtentForCurrentThread[6] = { 0, clipRectangleSize[0] - 1, 0, clipRectangleSize[1] - 1, 0, 0 };

			PasteSliceIntoVolumeInsertSliceParams paste_slice_info;
			paste_slice_info.outData = out_volume;
			paste_slice_info.outPtr = out_volume->GetBufferPointer();
			paste_slice_info.accPtr = acummulation_buffer->GetBufferPointer();
			paste_slice_info.clipRectangleOrigin = &clipRectangleOrigin[0];
			paste_slice_info.clipRectangleSize = &clipRectangleSize[0];
			paste_slice_info.inExt = inputFrameExtentForCurrentThread;
			paste_slice_info.interpolationMode = interpolation_strategy;
			paste_slice_info.compoundingMode = compounding_strategy;
			paste_slice_info.accOverflowCount = &accOverflow;
			paste_slice_info.pixelRejectionThreshold = 0;
			paste_slice_info.image_number = 0;

			Eigen::Matrix4d ref_to_image;
			ref_to_image(3, 0) = 0.0;
			ref_to_image(3, 1) = 0.0;
			ref_to_image(3, 2) = 0.0;
			ref_to_image(3, 3) = 1.0;

			// cicle throught all frames and insert
			// them in the output buffer, one at a time

			for (auto img : frame_data) {

				paste_slice_info.image_number += 1;

				itk::Matrix<double> image_orientation = img->GetDirection();
				itk::Point<double> image_origin = img->GetOrigin();

				ref_to_image(0, 0) = image_orientation[0][0];
				ref_to_image(1, 0) = image_orientation[1][0];
				ref_to_image(2, 0) = image_orientation[2][0];

				ref_to_image(0, 1) = image_orientation[0][1];
				ref_to_image(1, 1) = image_orientation[1][1];
				ref_to_image(2, 1) = image_orientation[2][1];

				ref_to_image(0, 2) = image_orientation[0][2];
				ref_to_image(1, 2) = image_orientation[1][2];
				ref_to_image(2, 2) = image_orientation[2][2];

				ref_to_image(0, 3) = image_origin[0];
				ref_to_image(1, 3) = image_origin[1];
				ref_to_image(2, 3) = image_origin[2];

				// The matrix is the transformation of the 
				// origin of the output volume (1) to the 
				// origin of the input image (2). This is 
				// given by T02=T01*T12, and by premultiplying 
				// by T10=inverse(T01) we obtain T12=inverse(T01)*T02
				Eigen::Matrix4d output_to_origin = output_to_ref * ref_to_image;

				curan::utils::console->info("Slice added");

				paste_slice_info.inData = img;
				paste_slice_info.inPtr = img->GetBufferPointer();
				paste_slice_info.matrix = output_to_origin;

				UnoptimizedInsertSlice(&paste_slice_info);

			};

			frame_data.clear();
		};

		void VolumeReconstructor::AddFrame(OutputType::Pointer image_pointer)
		{
			frame_data.push_back(image_pointer);
		};

		void VolumeReconstructor::AddFrames(std::vector<OutputType::Pointer>& images_vector)
		{
			frame_data.insert(std::end(frame_data), std::begin(images_vector), std::end(images_vector));
		}
		void VolumeReconstructor::SetOutputSpacing(OutputType::SpacingType in_output_spacing)
		{
			output_spacing = in_output_spacing;
		}
		void VolumeReconstructor::SetClippingBounds(std::array<double, 2> inclipRectangleOrigin, std::array<double, 2> inclipRectangleSize)
		{
			clipRectangleOrigin = inclipRectangleOrigin;
			clipRectangleSize = inclipRectangleSize;
		}

		void VolumeReconstructor::GetOutputPointer(OutputType::Pointer& pointer_to_be_changed)
		{
			pointer_to_be_changed = out_volume;
		};

		void VolumeReconstructor::SetFillStrategy(FillingStrategy strategy)
		{
			fillType = strategy;
		};

		void VolumeReconstructor::AddKernelDescritor(KernelDescriptor descriptor)
		{
			if (fillType == descriptor.fillType) {
				descriptor.Allocate();
				kernels.push_back(descriptor);
			}
		};

		void VolumeReconstructor::FillHoles()
		{
			CharPixelType* inVolPtr = out_volume->GetBufferPointer();
			ShortPixelType* accPtr = acummulation_buffer->GetBufferPointer();

			//we need to create the output volume where the 
			//voxels will be placed after the filling procedure is over
			itk::ImageDuplicator<InternalImageType>::Pointer duplicator = itk::ImageDuplicator<InternalImageType>::New();
			duplicator->SetInputImage(out_volume);
			duplicator->Update();
			InternalImageType::Pointer filled_volume = duplicator->GetOutput();
			CharPixelType* outPtr = filled_volume->GetBufferPointer();

			auto outdata_ROI = out_volume->GetLargestPossibleRegion();
			auto size_out = outdata_ROI.GetSize();
			auto start_out = outdata_ROI.GetIndex();

			uint64_t outExt[6];
			outExt[0] = start_out[0];
			outExt[1] = size_out[0] - 1;

			outExt[2] = start_out[1];
			outExt[3] = size_out[1] - 1;

			outExt[4] = start_out[2];
			outExt[5] = size_out[2] - 1;

			// get increments for volume and for accumulation buffer
			uint64_t byteIncVol[3] = { 0 }; //x,y,z

			int idx;
			uint64_t incr = INPUT_COMPONENTS;

			for (idx = 0; idx < 3; ++idx)
			{
				byteIncVol[idx] = incr;
				incr *= (outExt[idx * 2 + 1] - outExt[idx * 2] + 1);
			}

			// this will store the position of the pixel being looked at currently
			uint64_t currentPos[3]; //x,y,z

			uint64_t numVolumeComponents = INPUT_COMPONENTS;

			// Set interpolation method - nearest neighbor or trilinear
			bool (*apply)(CharPixelType * inputData,
				unsigned short* accData,
				uint64_t * inputOffsets,
				uint64_t * bounds,
				uint64_t * wholeExtent,
				uint64_t * thisPixel,
				CharPixelType & returnVal,
				const KernelDescriptor * descriptor) = NULL;

			switch (fillType)
			{
			case FillingStrategy::GAUSSIAN:
				apply = &ApplyGaussian;
				break;
			case FillingStrategy::GAUSSIAN_ACCUMULATION:
				apply = &ApplyGaussianAccumulation;
				break;
			case FillingStrategy::DISTANCE_WEIGHT_INVERSE:
				apply = &ApplyDistanceWeightInverse;
				break;
			case FillingStrategy::NEAREST_NEIGHBOR:
				apply = &ApplyNearestNeighbor;
				break;
			case FillingStrategy::STICK:
				apply = &ApplySticks;
				break;
			default:
			{
				std::string s = "Unknown interpolation mode: " + std::to_string(fillType);
				curan::utils::console->info(s);
				return;
			}
			}

			// iterate through each voxel. When the accumulation buffer is 0, fill that hole, and continue.
			for (currentPos[2] = outExt[4]; currentPos[2] <= outExt[5]; currentPos[2]++)
			{
				for (currentPos[1] = outExt[2]; currentPos[1] <= outExt[3]; currentPos[1]++)
				{
					for (currentPos[0] = outExt[0]; currentPos[0] <= outExt[1]; currentPos[0]++)
					{
						// accumulator index should not depend on which individual component is being interpolated
						int accIndex = (currentPos[0] * byteIncVol[0]) + (currentPos[1] * byteIncVol[1]) + (currentPos[2] * byteIncVol[2]);
						if (accPtr[accIndex] == 0) // if not hit by accumulation during vtkIGSIOPasteSliceIntoVolume
						{
							bool result(false);
							for (const auto kernel : kernels)
							{
								result = apply(inVolPtr, accPtr, byteIncVol, outExt, outExt, currentPos, outPtr[accIndex], &kernel);
								if (result) {
									break;
								} // end checking interpolation success
							}
						}
						else // if hit, just use the apparent value
						{
							int volCompIndex = (currentPos[0] * byteIncVol[0]) + (currentPos[1] * byteIncVol[1]) + (currentPos[2] * byteIncVol[2]);
							outPtr[volCompIndex] = inVolPtr[volCompIndex];
						} // end accumulation check
					} // end x loop
				} // end y loop
			} // end z loop

			out_volume = filled_volume;
		};


		bool VolumeReconstructor::UpdateInternalBuffers()
		{
			static gte::OrientedBox3<double> previous_box;

			if (previous_box != volumetric_bounding_box)
			{
				previous_box = volumetric_bounding_box;

				if (!volumes_initiated) {

					OutputType::IndexType output_start;
					output_start[0] = 0;
					output_start[1] = 0;
					output_start[2] = 0;

					OutputType::SizeType output_size;
					output_size[0] = std::ceil(volumetric_bounding_box.extent[0] * 2 / output_spacing[0]);
					output_size[1] = std::ceil(volumetric_bounding_box.extent[1] * 2 / output_spacing[1]);
					output_size[2] = std::ceil(volumetric_bounding_box.extent[2] * 2 / output_spacing[2]);

					OutputType::RegionType output_region;
					output_region.SetSize(output_size);
					output_region.SetIndex(output_start);

					gte::Vector<3, double> origin_gte = volumetric_bounding_box.center
						- volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0]
						- volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1]
						- volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];

					OutputType::PointType output_origin;
					output_origin[0] = origin_gte[0];
					output_origin[1] = origin_gte[1];
					output_origin[2] = origin_gte[2];

					OutputType::DirectionType output_directorion;
					output_directorion[0][0] = volumetric_bounding_box.axis[0][0];
					output_directorion[1][0] = volumetric_bounding_box.axis[1][0];
					output_directorion[2][0] = volumetric_bounding_box.axis[2][0];

					output_directorion[0][1] = volumetric_bounding_box.axis[0][1];
					output_directorion[1][1] = volumetric_bounding_box.axis[1][1];
					output_directorion[2][1] = volumetric_bounding_box.axis[2][1];

					output_directorion[0][2] = volumetric_bounding_box.axis[0][2];
					output_directorion[1][2] = volumetric_bounding_box.axis[1][2];
					output_directorion[2][2] = volumetric_bounding_box.axis[2][2];

					out_volume->SetRegions(output_region);
					out_volume->SetOrigin(output_origin);
					out_volume->SetSpacing(output_spacing);
					out_volume->SetDirection(output_directorion);
					out_volume->Allocate(true);

					acummulation_buffer->SetRegions(output_region);
					acummulation_buffer->SetOrigin(output_origin);
					acummulation_buffer->SetSpacing(output_spacing);
					acummulation_buffer->SetDirection(output_directorion);
					acummulation_buffer->Allocate(true);

					volumes_initiated = true;
					return true;
				}
				else {

					gte::Vector<3, double> origin_gte = volumetric_bounding_box.center
						- volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0]
						- volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1]
						- volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];

					ResamplerOutput::Pointer resampler = ResamplerOutput::New();

					itk::Matrix<double> transform;
					transform[0][0] = volumetric_bounding_box.axis[0][0];
					transform[1][0] = volumetric_bounding_box.axis[1][0];
					transform[2][0] = volumetric_bounding_box.axis[2][0];

					transform[0][1] = volumetric_bounding_box.axis[0][1];
					transform[1][1] = volumetric_bounding_box.axis[1][1];
					transform[2][1] = volumetric_bounding_box.axis[2][1];

					transform[0][2] = volumetric_bounding_box.axis[0][2];
					transform[1][2] = volumetric_bounding_box.axis[1][2];
					transform[2][2] = volumetric_bounding_box.axis[2][2];

					itk::AffineTransform<double>::Pointer affine_transform = itk::AffineTransform<double>::New();
					resampler->SetTransform(affine_transform);

					itk::NearestNeighborInterpolateImageFunction<OutputType>::Pointer interpolator = itk::NearestNeighborInterpolateImageFunction<OutputType>::New();
					resampler->SetInterpolator(interpolator);

					OutputType::SizeType output_size;
					output_size[0] = std::ceil(volumetric_bounding_box.extent[0] * 2 / output_spacing[0]);
					output_size[1] = std::ceil(volumetric_bounding_box.extent[1] * 2 / output_spacing[1]);
					output_size[2] = std::ceil(volumetric_bounding_box.extent[2] * 2 / output_spacing[2]);

					resampler->SetSize(output_size);
					resampler->SetDefaultPixelValue(0);
					resampler->SetOutputSpacing(output_spacing);
					resampler->SetOutputOrigin(origin_gte[0]);
					resampler->SetOutputDirection(transform);
					resampler->SetOutputStartIndex({ 0,0,0 });
					resampler->SetInput(out_volume);
					OutputType::Pointer new_output;

					try {
						resampler->Update();
						new_output = resampler->GetOutput();
					}
					catch (...) {
						return false;
					}

					out_volume = new_output;

					itk::NearestNeighborInterpolateImageFunction<AccumulatorType>::Pointer accumulator_interpolator = itk::NearestNeighborInterpolateImageFunction<AccumulatorType>::New();
					ResamplerAccumulator::Pointer resampler_accumulator = ResamplerAccumulator::New();
					resampler_accumulator->SetTransform(affine_transform);
					resampler_accumulator->SetInterpolator(accumulator_interpolator);
					resampler_accumulator->SetSize(output_size);
					resampler_accumulator->SetDefaultPixelValue(0);
					resampler_accumulator->SetOutputSpacing(output_spacing);
					resampler_accumulator->SetOutputOrigin(origin_gte[0]);
					resampler_accumulator->SetOutputDirection(transform);
					resampler_accumulator->SetOutputStartIndex({ 0,0,0 });
					resampler_accumulator->SetInput(acummulation_buffer);
					AccumulatorType::Pointer new_acummulation_buffer;

					try {
						resampler_accumulator->Update();
						new_acummulation_buffer = resampler_accumulator->GetOutput();
					}
					catch (...) {
						return false;
					}

					acummulation_buffer = new_acummulation_buffer;

					return true;
				}
				return true;
			};
		}

		VolumeReconstructor::KernelDescriptor::KernelDescriptor()
		{
		}

		VolumeReconstructor::KernelDescriptor::~KernelDescriptor()
		{
			if (sticksList != nullptr)
				delete[] sticksList;

			if (kernel != nullptr)
				delete[] kernel;
		}

		VolumeReconstructor::KernelDescriptor::KernelDescriptor(const KernelDescriptor& other)
		{
			stickLengthLimit = other.stickLengthLimit;
			numSticksToUse = other.numSticksToUse;    // the number of sticks to use in averaging the final voxel value
			numSticksInList = other.numSticksInList;         // the number of sticks in sticksList
			fillType = other.fillType;
			size = other.size;
			stdev = other.stdev;
			minRatio = other.minRatio;

			if (other.sticksList != nullptr) {
				if (sticksList != nullptr)
					delete[] sticksList;
				size_t sticksList_size = 39;
				sticksList = new int[sticksList_size];
				std::memcpy(sticksList, other.sticksList, sticksList_size);
			}

			if (other.kernel != nullptr) {
				if (kernel != nullptr)
					delete[] kernel;
				size_t kernel_size = size * size * size;
				kernel = new float[kernel_size];
				std::memcpy(kernel, other.kernel, kernel_size);
			}
		}

		VolumeReconstructor::KernelDescriptor& VolumeReconstructor::KernelDescriptor::operator=(const KernelDescriptor& other)
		{
			if (this != &other) {
				stickLengthLimit = other.stickLengthLimit;
				numSticksToUse = other.numSticksToUse;    // the number of sticks to use in averaging the final voxel value
				numSticksInList = other.numSticksInList;         // the number of sticks in sticksList
				fillType = other.fillType;
				size = other.size;
				stdev = other.stdev;
				minRatio = other.minRatio;

				if (other.sticksList != nullptr) {
					if (sticksList != nullptr)
						delete[] sticksList;
					size_t sticksList_size = 39;
					sticksList = new int[sticksList_size];
					std::memcpy(sticksList, other.sticksList, sticksList_size);
				}

				if (other.kernel != nullptr) {
					if (kernel != nullptr)
						delete[] kernel;
					size_t kernel_size = size * size * size;
					kernel = new float[kernel_size];
					std::memcpy(kernel, other.kernel, kernel_size);
				}
			}
			return *this;
		}

		void VolumeReconstructor::KernelDescriptor::Allocate()
		{
			switch (fillType)
			{
			case FillingStrategy::DISTANCE_WEIGHT_INVERSE:
			{
				if (kernel != nullptr)
					delete[] kernel;
				kernel = new float[size * size * size];

				float range = (size - 1) / 2.;
				int index(0);
				for (int z = 0; z < size; z++)
				{
					for (int y = 0; y < size; y++)
					{
						for (int x = 0; x < size; x++)
						{
							float xD = (x - range);
							float yD = (y - range);
							float zD = (z - range);
							float distance = sqrt(xD * xD + yD * yD + zD * zD); // use euclidean distance
							if (distance) { // avoid division by zero
								float invD = fabsf(1.0f / distance);
								kernel[index] = invD;
							}
							else {
								kernel[index] = 0.0f; // center shouldn't have any weight anyway
							}
							index++;
						}
					}
				}

			}
			break;
			case FillingStrategy::GAUSSIAN:
			{
				if (kernel != nullptr)
					delete[] kernel;
				kernel = new float[size * size * size];

				float range = (size - 1) / 2.;
				const double Pi = 3.1415926535897932384626433832795;

				// divisors in the exponent
				float divisor = stdev * stdev * 2.0;

				// divisor in the non-exponent
				float termDivisor = pow(2 * Pi, 3. / 2) * pow(stdev, 3);

				int index(0);
				for (int z = 0; z < size; z++)
				{
					for (int y = 0; y < size; y++)
					{
						for (int x = 0; x < size; x++)
						{
							double xExp = pow((double)x - range, 2) / divisor;
							double yExp = pow((double)y - range, 2) / divisor;
							double zExp = pow((double)z - range, 2) / divisor;
							double eExp = -(xExp + yExp + zExp);

							float calcVal = (1.0f) / (termDivisor)*exp(eExp);
							kernel[index] = calcVal;
							index++;
						}
					}
				}
			}
			break;
			case FillingStrategy::GAUSSIAN_ACCUMULATION:
			{
				if (kernel != nullptr)
					delete[] kernel;
				kernel = new float[size * size * size];

				float range = (size - 1) / 2.;
				const double Pi = 3.1415926535897932384626433832795;

				// divisors in the exponent
				float divisor = stdev * stdev * 2.0;

				// divisor in the non-exponent
				float termDivisor = pow(2 * Pi, 3. / 2) * pow(stdev, 3);

				int index(0);
				for (int z = 0; z < size; z++)
				{
					for (int y = 0; y < size; y++)
					{
						for (int x = 0; x < size; x++)
						{
							double xExp = pow((double)x - range, 2) / divisor;
							double yExp = pow((double)y - range, 2) / divisor;
							double zExp = pow((double)z - range, 2) / divisor;
							double eExp = -(xExp + yExp + zExp);

							float calcVal = (1.0f) / (termDivisor)*exp(eExp);
							kernel[index] = calcVal;
							index++;
						}
					}
				}
			}
			break;
			case FillingStrategy::NEAREST_NEIGHBOR:
			{
				// we do not need to allocate anything for the nearest neighbor algorithm
			}
			break;
			case FillingStrategy::STICK:
			{
				numSticksInList = 13;
				if (sticksList != nullptr)
					delete[] sticksList;
				sticksList = new int[39];

				// 1x1, 2x0
				sticksList[0] = 1; sticksList[1] = 0; sticksList[2] = 0; // x, y, z
				sticksList[3] = 0; sticksList[4] = 1; sticksList[5] = 0;
				sticksList[6] = 0; sticksList[7] = 0; sticksList[8] = 1;

				// 2x1, 1x0
				sticksList[9] = 1; sticksList[10] = 1; sticksList[11] = 0;
				sticksList[12] = 1; sticksList[13] = 0; sticksList[14] = 1;
				sticksList[15] = 0; sticksList[16] = 1; sticksList[17] = 1;
				// 1x1, 1x-1, 1x0
				sticksList[18] = 1; sticksList[19] = -1; sticksList[20] = 0;
				sticksList[21] = 1; sticksList[22] = 0; sticksList[23] = -1;
				sticksList[24] = 0; sticksList[25] = 1; sticksList[26] = -1;

				// 3x1
				sticksList[27] = 1; sticksList[28] = 1; sticksList[29] = 1;
				// 2x1, 1x-1
				sticksList[30] = -1; sticksList[31] = 1; sticksList[32] = 1;
				sticksList[33] = 1; sticksList[34] = -1; sticksList[35] = 1;
				sticksList[36] = -1; sticksList[37] = -1; sticksList[38] = 1;

			}
			break;
			default:
				// should we return an error?
				break;
			}

		}
	}
}