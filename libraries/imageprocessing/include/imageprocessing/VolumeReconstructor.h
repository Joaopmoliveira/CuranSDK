#include "ImageProcessingDefinitions.h"

namespace curan{
    namespace image{
        /*
		This class of code is a copy of the code provided by
		IGSIO repository on youtube. Becase we do not wish to
		introduce
		*/
		class VolumeReconstructor
		{
		public:
			using output_type = itk::Image<CharPixelType, Dimension3D>;
			using accumulator_type = itk::Image<ShortPixelType, Dimension3D>;
			using resampler_output = itk::ResampleImageFilter<OutputType, OutputType>;
			using resampler_accumulator = itk::ResampleImageFilter<AccumulatorType, AccumulatorType>;

			// With this code we always assume 
			// that the images are greyscale
			// therefore there is a single value
			// to manipulate
			static constexpr int INPUT_COMPONENTS = 1;

			VolumeReconstructor();
			~VolumeReconstructor();

			enum Interpolation{
				NEAREST_NEIGHBOR_INTERPOLATION,
				LINEAR_INTERPOLATION
			};

			enum Compounding{
				UNDEFINED_COMPOUNDING_MODE,
				LATEST_COMPOUNDING_MODE,
				MAXIMUM_COMPOUNDING_MODE,
				MEAN_COMPOUNDING_MODE,
			};

			enum FillingStrategy
			{
				GAUSSIAN,
				GAUSSIAN_ACCUMULATION,
				DISTANCE_WEIGHT_INVERSE,
				NEAREST_NEIGHBOR,
				STICK
			};

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

			void update();

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
			void add_frame(output_type::Pointer);

			/*
			When one has all the frames which will be used to
			for the volumetric reconstruction, they can provide all 
			images at once, thus removing needless time spent submitting 
			the images.
			*/
			void add_frames(std::vector<output_type::Pointer>&);

			/*
			One should always specify the output spacing 
			before using the volume reconstructor, or 
			else it will be initialized with a default spacing 
			of {1.0 mm, 1.0 mm, 1.0 mmm} os spacing.
			*/
			void set_output_spacing(output_type::SpacingType in_output_spacing);

			/*
			Define the clipping bounds of the input images. 
			By default the clipping bounds are set to zero.
			*/
			void set_clipping_bounds(std::array<double, 2> inclipRectangleOrigin, std::array<double, 2> inclipRectangleSize);

			/*
			Obtain the pointer to the 3D itk volume reconstructed
			and filled under the hood with this class.
			*/
			void get_output_pointer(output_type::Pointer& pointer_to_be_changed);

			/*
			The fill strategy to be used when the FillHoles
			method is called.
			*/
			void set_fill_strategy(FillingStrategy strategy);

			/*
			The kernel descriptor is the structure which defines
			the kernel used to fill the empty voxels left in the 
			output volume.
			*/
			void add_kernel_descritor(KernelDescriptor descriptor);

			/*
			Once all the data has been inserted into the
			output volume the last step of this procedure
			relies on filling the empty spaces left in
			the output volume which were set with the
			default value.
			*/
			void fill_holes();

		private:

			/*
			Method used to resize the output buffers as required by 
			Update method, and the current images contained in the frame
			data container. The code will go through the stored 
			bounding box, check if the internall buffer must be enarlged 
			and reshape the current buffer into the new allocated buffer, 
			if required.
			*/
			bool update_internal_buffers();

			Interpolation interpolation_strategy = Interpolation::NEAREST_NEIGHBOR_INTERPOLATION;
			Compounding compounding_strategy = Compounding::LATEST_COMPOUNDING_MODE;

			std::array<double, 2> clipRectangleOrigin = { 0.0,0.0 }; // array size 2
			std::array<double, 2> clipRectangleSize = { 0.0,0.0 };; // array size 2
			std::vector<output_type::Pointer> frame_data;

			output_type::SpacingType output_spacing;
			output_type::Pointer out_volume;
			accumulator_type::Pointer acummulation_buffer;

			bool volumes_initiated = false;
			gte::OrientedBox3<double> volumetric_bounding_box;

			//data related with volume filling
			std::vector<KernelDescriptor> kernels;
			FillingStrategy fillType;
		};
    }
}