#ifndef PkLibrary_h_DEFINED
#define PkLibrary_h_DEFINED

#include <map>
#include <tuple>
#include <sigslot/signal.hpp>

#include "itkImage.h"
#include "itkImageDuplicator.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "itkResampleImageFilter.h"
#include "itkAffineTransform.h"
#include "itkExtractImageFilter.h"
#include "itkImageSliceConstIteratorWithIndex.h"
#include "itkNearestNeighborInterpolateImageFunction.h"
#include "itkImageLinearIteratorWithIndex.h"
#include "itkRegionOfInterestImageFilter.h"
#include "itkMetaDataObject.h"
#include "itkTransform.h"
#include <string>
#include "DkUtilities.h"
#include "itkThresholdImageFilter.h"
#include "itkCannyEdgeDetectionImageFilter.h"
#include "Mathematics/ConvexHull3.h"
#include "Mathematics/ArbitraryPrecision.h"
#include "Mathematics/MinimumVolumeBox3.h"
#include <filesystem>

#define SK_VULKAN
#include "include\core\SkColor.h"
#include "include\core\SkColorFilter.h"
#include "include\core\SkEncodedImageFormat.h"
#include "include\core\SkImage.h"
#include "include\core\SkImageEncoder.h"
#include "include\core\SkImageFilter.h"
#include "include\core\SkImageInfo.h"
#include "include\core\SkPaint.h"
#include "include\core\SkPixelRef.h"
#include "include\core\SkPixmap.h"

#include "Mathematics/ConvexHull3.h"
#include "Mathematics/ArbitraryPrecision.h"
#include "Mathematics/MinimumVolumeBox3.h"

namespace curan {
	namespace image {
		//*******************************************
	/*
	Main structures of the library.

	The logic of this implementation is simple.

	There is a PkProcess variable that is common for all jobs. When a given portion
	of the code locally wants to load or unload an image it requests a
	reference to the PkProcess structure, and procedes to use it to process all requests.

	The PkProcess variable keeps all volumes loaded, so that no information is lost.
	When the application terminates all volumes are eliminated. If the user wishes to
	release some memory, they can request to eliminate volumes from memory.

	Internally the PkProcess keps a map where integers are associated with a given pointer
	to a memory location. When the user wants to use the memory, he provides a structure where
	image information will be layed out for him.

	A natural use case would be the following:

	assume you want to initialize the PkProcess. If that is the case then one can call
	the PkCreateImageProcessor function which will instantiate a unique object PkProcess

	void initialization()
	{
		PkResult res;
		PkImageManager* manager = nullptr;
		PkCreateVolumeProc create_proc;
		res = PkCreateImageProcessor(&manager, create_proc);
	}


	Suppose you now want to upload into memory an image. To do this we need a pointer to
	the initialized PkProcess which we can obtain by calling PkGetImageProcessor. After
	this call the pointer upload_process will be pointing towards the correct PkProcess.
	To finish uploading the file one can now call the function PkLoadVolume which will receive
	a pointer to a PkProcess and the information associated with the file we wish to read

	void upload()
	{
		PkResult res;
		PkImageManager* manager = nullptr;
		res = PkGetImageProcessor(&manager);

		if (res == PkResult::PK_FAILED_TO_FIND_BACKEND)
			return;

		PkLoadVolumeInfo load_vol;
		load_vol.filetype = PkFileType::PK_SINGLE_FILE;
		load_vol.identifier = 200;
		load_vol.path = "C:\\image_dicom.dcm";
		res = PkLoadVolume(manager, load_vol);
	}

	Suppose now that after the volume has been uploaded we wish to use it in another function.
	Once again we need to obtain the pointer to the PkProcess throught PkGetImageProcessor()
	and then we can get the pointer to the itk image throught PkGetVolume() and now we have a
	PkVolume which contains the image itk pointer

	void use_volume()
	{
		PkResult res;
		PkImageManager* manager = nullptr;
		res = PkGetImageProcessor(&manager);

		PkGetVolumeInfo get_vol;
		get_vol.identifier = 200;
		PkVolume vol;
		res = PkGetVolume(manager, get_vol, vol);
	}
	*/
	//********************************************************************
		using ShortPixelType = unsigned short;
		using CharPixelType = unsigned char;

		constexpr unsigned int Dimension3D = 3;
		using InputDICOMImageType = itk::Image<ShortPixelType, Dimension3D>;

		// The dimensions of the Internal image type are 3, 
		// not because it is required by the image itself,
		// which are 2D, but by the because it is the only
		// way to have the position of the 2D image of
		// individual pixels in 3D space. (This is required
		// to some algorithms we wish to implement such as 
		// volume reconstruction, volume mapping between 
		// two images and so on)
		using InternalImageType = itk::Image<CharPixelType, Dimension3D>;
		using RescaleType = itk::RescaleIntensityImageFilter<InputDICOMImageType, InputDICOMImageType>;
		using FilterType = itk::CastImageFilter<InputDICOMImageType, InternalImageType>;
		using ImageIOType = itk::GDCMImageIO;
		using SlicerType = itk::ResampleImageFilter<InternalImageType, InternalImageType>;
		using IterateType = itk::ExtractImageFilter<InternalImageType, InternalImageType>;
		using DictionaryType = itk::MetaDataDictionary;
		using MetaDataStringType = itk::MetaDataObject<std::string>;

		/*
		Enumeration of all possible output results
		from the PkLibrary functions.
		*/
		enum class Result {
			PK_SUCCESS,
			PK_FAILED_TO_FIND_BACKEND,
			PK_PATH_FAILURE,
			PK_VOLUME_NOT_PRESENT,
			PK_VOLUME_PRESENT,
		};

		/*
		The PkStudy is the logical unit of the application.
		Once the user request to upload a given file or directory
		the software will identify all studies and load a vector
		containing a pointer for each 2D image.
		*/
		struct Study {
			std::vector<InternalImageType::Pointer> study_img;
			std::string individual_name;
			std::string image_number;
			sk_sp<SkImage> image;
		};

		/*
		Wrapper around an ITK 3D volume which
		has a depth of 8bits per pixel.
		This volume must be reconstructed from
		the loaded study.
		*/
		struct Volume {
			std::string individual_name;
			InternalImageType::Pointer volume;
		};

		/*
		The PkImageManager internally contains
		a map pointing towards ITK images, where
		each image is associated with a number
		which must be used to obtain a pointer
		towards said image.
		*/
		class StudyManager {
		public:

			/*
			The get method returns the application wide study manager
			which can be used in different ways depending on the wishes of the 
			developer.
			*/
			static StudyManager* Get();

			/*
			* The loaded study call member is a collection of slots which 
			whish to be warned when a study is loaded into memory. Whenever thats
			the case, the submited method will be called and it will return an
			integer with the index of the new study which in turn can be used to
			query for that particular study.
			*/
			sigslot::signal<int*> loaded_study_call;

			/*
			The PkLoadVolume function receives a pointer to the
			PkImageManager and a reference to a PkLoadVolumeInfo
			structure. It searches internally for the volumes already
			in memory and makes sure that the provided identifier is
			unique. If it's not a PkResult with an error is returned.
			*/
			Result LoadStudies(std::vector<std::filesystem::path> paths);

			/*
			The PkUnloadVolume function removes a given image from
			memory. If to much memory has been consumed by the
			application one can erase images from the Image
			manager to solve the problem.
			*/
			Result UnloadStudy(uint64_t identifier);

			/*
			The PkGetVolume function returns a PkVolume stored
			inside the Image manager associated with a given
			identifier.
			*/
			Result GetStudy(uint64_t identifier, Study& out_vol);

			/*
			Obtain a pointer to the vector containing the 
			previews of all the current loaded studies.
			*/
			void GetStudies(std::map<uint64_t, Study>& previews);

		private:
			static uint32_t identifier;

			StudyManager();

			~StudyManager();

			std::map<uint64_t, Study> study_container;

			std::mutex mut;
		};

		/*
		This class of code is a copy of the code provided by
		IGSIO repository on youtube. Becase we do not wish to
		introduce
		*/
		class VolumeReconstructor
		{
		public:
			using OutputType = itk::Image<CharPixelType, Dimension3D>;
			using AccumulatorType = itk::Image<ShortPixelType, Dimension3D>;
			using ResamplerOutput = itk::ResampleImageFilter<OutputType, OutputType>;
			using ResamplerAccumulator = itk::ResampleImageFilter<AccumulatorType, AccumulatorType>;
			// With this code we always assume 
			// that the images are greyscale
			// therefore there is a single value
			// to manipulate
			static constexpr int INPUT_COMPONENTS = 1;

			VolumeReconstructor();
			~VolumeReconstructor();

			/*
			Strategy used to aproximate the values 
			of voxels according to the surrounding 
			pixels
			*/
			enum Interpolation{
				NEAREST_NEIGHBOR_INTERPOLATION,
				LINEAR_INTERPOLATION
			};

			/*
			Strategy used to accumulate the previous 
			pixel values already inserted in the
			buffer
			*/
			enum Compounding{
				UNDEFINED_COMPOUNDING_MODE,
				LATEST_COMPOUNDING_MODE,
				MAXIMUM_COMPOUNDING_MODE,
				MEAN_COMPOUNDING_MODE,
			};

			/*
			The fill strategy is used 
			*/
			enum FillingStrategy
			{
				GAUSSIAN,
				GAUSSIAN_ACCUMULATION,
				DISTANCE_WEIGHT_INVERSE,
				NEAREST_NEIGHBOR,
				STICK
			};

			/*
			The kernel descriptor is used to pass 
			information to the filling algorithms which
			require specific information to function 
			properlly. Namelly the functions:
			-ApplyNearestNeighbor
			-ApplyDistanceWeightInverse
			-ApplyGaussian
			-ApplyGaussianAccumulation
			-ApplySticks
			Since a number of kernels can be used to 
			fill the empty values, we use this struct to
			aglomerate information specific to each 
			kernel.
			*/
			struct KernelDescriptor
			{
				//constructor
				KernelDescriptor();

				//destructor
				~KernelDescriptor();
				
				//copy constructor
				KernelDescriptor(const KernelDescriptor& other);

				//copy assignment
				KernelDescriptor& operator=(const KernelDescriptor& other);

				//move constructor

				//move assignment

				//data related with volume filling
				int stickLengthLimit= -1;
				int numSticksToUse= -1;    // the number of sticks to use in averaging the final voxel value
				int numSticksInList= -1;         // the number of sticks in sticksList
				
				VolumeReconstructor::FillingStrategy fillType = VolumeReconstructor::FillingStrategy::GAUSSIAN;
				int size = -1;
				float stdev = -1;
				float minRatio = -1;

				int* sticksList = nullptr; // triples each corresponding to a stick orientation
				float* kernel = nullptr; // stores the gaussian weights for this kernel


				void Allocate();
			};

			/*
			* If this algorithm is supposed to run in
			real time, we should allow the developer to
			control when we wishes to update the current
			reconstructed volume with the new ultrasound
			images introduced with the add frame method.
			*/
			void Update();

			/*
			When you start adding frames, make sure they
			all share the same frame of reference. If
			they do not then obvious mistakes will be made
			while reconstructing the volume. It is even
			possible to crash the computer because a lot
			of memory might be requested to create a large
			enough volume to contain the missplaced images
			in space.
			*/
			void AddFrame(OutputType::Pointer);

			/*
			When one has all the frames which will be used to
			for the volumetric reconstruction, they can provide all 
			images at once, thus removing needless time spent submitting 
			the images.
			*/
			void AddFrames(std::vector<OutputType::Pointer>&);

			/*
			One should always specify the output spacing 
			before using the volume reconstructor, or 
			else it will be initialized with a default spacing 
			of {1.0 mm, 1.0 mm, 1.0 mmm} os spacing.
			*/
			void SetOutputSpacing(OutputType::SpacingType in_output_spacing);

			/*
			Define the clipping bounds of the input images. 
			By default the clipping bounds are set to zero.
			*/
			void SetClippingBounds(std::array<double, 2> inclipRectangleOrigin, std::array<double, 2> inclipRectangleSize);

			/*
			Obtain the pointer to the 3D itk volume reconstructed
			and filled under the hood with this class.
			*/
			void GetOutputPointer(OutputType::Pointer& pointer_to_be_changed);

			/*
			The fill strategy to be used when the FillHoles
			method is called.
			*/
			void SetFillStrategy(FillingStrategy strategy);

			/*
			The kernel descriptor is the structure which defines
			the kernel used to fill the empty voxels left in the 
			output volume.
			*/
			void AddKernelDescritor(KernelDescriptor descriptor);

			/*
			Once all the data has been inserted into the
			output volume the last step of this procedure
			relies on filling the empty spaces left in
			the output volume which were set with the
			default value.
			*/
			void FillHoles();

		private:

			/*
			Method used to resize the output buffers as required by 
			Update method, and the current images contained in the frame
			data container. The code will go through the stored 
			bounding box, check if the internall buffer must be enarlged 
			and reshape the current buffer into the new allocated buffer, 
			if required.
			*/
			bool UpdateInternalBuffers();

			Interpolation interpolation_strategy = Interpolation::NEAREST_NEIGHBOR_INTERPOLATION;
			Compounding compounding_strategy = Compounding::LATEST_COMPOUNDING_MODE;

			std::array<double, 2> clipRectangleOrigin = { 0.0,0.0 }; // array size 2
			std::array<double, 2> clipRectangleSize = { 0.0,0.0 };; // array size 2
			std::vector<OutputType::Pointer> frame_data;

			OutputType::SpacingType output_spacing;
			OutputType::Pointer out_volume;
			AccumulatorType::Pointer acummulation_buffer;

			bool volumes_initiated = false;
			gte::OrientedBox3<double> volumetric_bounding_box;

			//data related with volume filling
			std::vector<KernelDescriptor> kernels;
			FillingStrategy fillType;
		};

		/*
		These are the parameters that are supplied to any given "InsertSlice" function, whether it be
		optimized or unoptimized.
		*/
		struct PasteSliceIntoVolumeInsertSliceParams
		{
			// information on the volume
			InternalImageType::Pointer outData;            // the output volume
			void* outPtr;                     // scalar pointer to the output volume over the output extent
			unsigned short* accPtr;           // scalar pointer to the accumulation buffer over the output extent
			InternalImageType::Pointer inData;             // input slice
			void* inPtr;                      // scalar pointer to the input volume over the input slice extent
			int* inExt;                       // array size 6, input slice extent (could have been split for threading)
			unsigned int* accOverflowCount;   // the number of voxels that may have error due to accumulation overflow

			// transform matrix for images -> volume
			Eigen::Matrix4d matrix;

			// details specified by the user RE: how the voxels should be computed
			VolumeReconstructor::Interpolation interpolationMode;   // linear or nearest neighbor
			VolumeReconstructor::Compounding compoundingMode;

			// parameters for clipping
			double* clipRectangleOrigin; // array size 2
			double* clipRectangleSize; // array size 2

			double pixelRejectionThreshold;
			int image_number;
		};

		// regarding these values, see comments at the top of this file by Thomas Vaughan
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
#endif