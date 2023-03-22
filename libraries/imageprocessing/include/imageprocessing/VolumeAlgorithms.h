#ifndef CURAN_IMAGE_ALGORITHMS_HEADER_FILE_
#define CURAN_IMAGE_ALGORITHMS_HEADER_FILE_



namespace curan {
	namespace image {
		namespace reconstruction {

			constexpr uint64_t ACCUMULATION_MULTIPLIER = 256;
			constexpr uint64_t ACCUMULATION_MAXIMUM = 65535;
			constexpr uint64_t ACCUMULATION_THRESHOLD = 65279; // calculate manually to possibly save on computation time
			constexpr double fraction1_256 = 1.0 / 256;
			constexpr double fraction255_256 = 255.0 / 256;

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

			/*
			Implements trilinear interpolation
			Does reverse trilinear interpolation. Trilinear interpolation would use
			the pixel values to interpolate something in the middle we have the
			something in the middle and want to spread it to the discrete pixel
			values around it, in an interpolated way
			Do trilinear interpolation of the input data 'inPtr' of extent 'inExt'
			at the 'point'.  The result is placed at 'outPtr'.
			If the lookup data is beyond the extent 'inExt', set 'outPtr' to
			the background color 'background'.
			The number of scalar components in the data is 'numscalars'
			*/
			int TrilinearInterpolation(const Eigen::Vector4d point,
				CharPixelType* inPtr,
				CharPixelType* outPtr,
				unsigned short* accPtr,
				int numscalars,
				VolumeReconstructor::Compounding compoundingMode,
				int outExt[6],
				uint64_t outInc[3],
				unsigned int* accOverflowCount);


			/*
			Non-optimized nearest neighbor interpolation.
			In the un-optimized version, each output voxel
			is converted into a set of look-up indices for the input data;
			then, the indices are checked to ensure they lie within the
			input data extent.
			In the optimized versions, the check is done in reverse:
			it is first determined which output voxels map to look-up indices
			within the input data extent.  Then, further calculations are
			done only for those voxels.  This means that 1) minimal work
			is done for voxels which map to regions outside of the input
			extent (they are just set to the background color) and 2)
			the inner loops of the look-up and interpolation are
			tightened relative to the un-uptimized version.
			Do nearest-neighbor interpolation of the input data 'inPtr' of extent
			'inExt' at the 'point'.  The result is placed at 'outPtr'.
			If the lookup data is beyond the extent 'inExt', set 'outPtr' to
			the background color 'background'.
			The number of scalar components in the data is 'numscalars'
			*/
			int NearestNeighborInterpolation(const Eigen::Vector4d point,
				CharPixelType* inPtr,
				CharPixelType* outPtr,
				unsigned short* accPtr,
				int numscalars,
				VolumeReconstructor::Compounding compoundingMode,
				int outExt[6],
				uint64_t outInc[3],
				unsigned int* accOverflowCount);

#define PIXEL_REJECTION_DISABLED (-DBL_MAX)
			bool PixelRejectionEnabled(double threshold);

			/*
			Actually inserts the slice - executes the filter for any type of data, without optimization
			Given an input and output region, execute the filter algorithm to fill the
			output from the input - no optimization.
			(this one function is pretty much the be-all and end-all of the filter)
			*/
			void UnoptimizedInsertSlice(PasteSliceIntoVolumeInsertSliceParams* insertionParams);


			bool ApplyNearestNeighbor(CharPixelType* inputData,            // contains the dataset being interpolated between
				unsigned short* accData, // contains the weights of each voxel
				uint64_t* inputOffsets, // contains the indexing offsets between adjacent x,y,z
				uint64_t* bounds,             // the boundaries of the thread
				uint64_t* wholeExtent,        // the boundaries of the volume, outputExtent
				uint64_t* thisPixel,          // The x,y,z coordinates of the voxel being calculated
				CharPixelType& returnVal,
				const VolumeReconstructor::KernelDescriptor* descrip);           // The value of the pixel being calculated (unknown);


			bool ApplyDistanceWeightInverse(CharPixelType* inputData,            // contains the dataset being interpolated between
				unsigned short* accData, // contains the weights of each voxel
				uint64_t* inputOffsets, // contains the indexing offsets between adjacent x,y,z
				uint64_t* bounds,             // the boundaries of the thread
				uint64_t* wholeExtent,        // the boundaries of the volume, outputExtent
				uint64_t* thisPixel,          // The x,y,z coordinates of the voxel being calculated
				CharPixelType& returnVal,
				const VolumeReconstructor::KernelDescriptor* descrip);           // The value of the pixel being calculated (unknown);


			bool ApplyGaussian(CharPixelType* inputData,            // contains the dataset being interpolated between
				unsigned short* accData, // contains the weights of each voxel
				uint64_t* inputOffsets, // contains the indexing offsets between adjacent x,y,z
				uint64_t* bounds,             // the boundaries of the thread
				uint64_t* wholeExtent,        // the boundaries of the volume, outputExtent
				uint64_t* thisPixel,          // The x,y,z coordinates of the voxel being calculated
				CharPixelType& returnVal,
				const VolumeReconstructor::KernelDescriptor* descrip);           // The value of the pixel being calculated (unknown);


			bool ApplyGaussianAccumulation(CharPixelType* inputData,            // contains the dataset being interpolated between
				unsigned short* accData, // contains the weights of each voxel
				uint64_t* inputOffsets, // contains the indexing offsets between adjacent x,y,z
				uint64_t* bounds,             // the boundaries of the thread
				uint64_t* wholeExtent,        // the boundaries of the volume, outputExtent
				uint64_t* thisPixel,          // The x,y,z coordinates of the voxel being calculated
				CharPixelType& returnVal,
				const VolumeReconstructor::KernelDescriptor* descrip);           // The value of the pixel being calculated (unknown);


			bool ApplySticks(CharPixelType* inputData,            // contains the dataset being interpolated between
				unsigned short* accData, // contains the weights of each voxel
				uint64_t* inputOffsets, // contains the indexing offsets between adjacent x,y,z
				uint64_t* bounds,             // the boundaries of the thread
				uint64_t* wholeExtent,        // the boundaries of the volume, outputExtent
				uint64_t* thisPixel,          // The x,y,z coordinates of the voxel being calculated
				CharPixelType& returnVal,
				const VolumeReconstructor::KernelDescriptor* descrip);           // The value of the pixel being calculated (unknown);
		}
	}
}

#endif