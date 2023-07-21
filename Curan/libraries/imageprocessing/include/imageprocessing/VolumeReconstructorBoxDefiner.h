#ifndef CURAN_VOLUME_RECONSTRUCTOR_BOX_DEFINER_HEADER_FILE_
#define CURAN_VOLUME_RECONSTRUCTOR_BOX_DEFINER_HEADER_FILE_

#include "ImageProcessingDefinitions.h"
#include "KernelDescriptor.h"
#include "Mathematics/ConvexHull3.h"
#include "Mathematics/ArbitraryPrecision.h"
#include "Mathematics/MinimumVolumeBox3.h"
#include "VolumeAlgorithms.h"

namespace curan{
    namespace image{
        /*
		This class of code is a copy of the code provided by
		IGSIO repository on youtube. Becase we do not wish to
		introduce
		*/
		class VolumeReconstructorBoxDefiner
		{
		public:
			using output_type = itk::Image<char_pixel_type, Dimension3D>;
			using accumulator_type = itk::Image<short_pixel_type, Dimension3D>;
			using resampler_output = itk::ResampleImageFilter<output_type, output_type>;
			using resampler_accumulator = itk::ResampleImageFilter<accumulator_type, accumulator_type>;
			using array_type = std::array<gte::Vector3<double>, 8>;

			VolumeReconstructorBoxDefiner();
			~VolumeReconstructorBoxDefiner();

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

		/*
			When one has all the frames which will be used to
			for the volumetric reconstruction, they can provide all 
			images at once, thus removing needless time spent submitting 
			the images.
			*/

			void add_frame(output_type::Pointer);

			void add_frames(std::vector<output_type::Pointer>&);
			
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
			void get_final_volume_vertices(array_type& box_data);

			/*
			The fill strategy to be used when the FillHoles
			method is called.
			*/

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

			std::array<double, 2> clipRectangleOrigin = { 0.0,0.0 }; // array size 2
			std::array<double, 2> clipRectangleSize = { 0.0,0.0 };; // array size 2
			std::vector<output_type::Pointer> frame_data;

			output_type::SpacingType output_spacing;
			output_type::Pointer out_volume;
			accumulator_type::Pointer acummulation_buffer;

			bool volumes_initiated = false;
			gte::OrientedBox3<double> volumetric_bounding_box;

			//data related with volume filling
			std::vector<reconstruction::KernelDescriptor> kernels;
			reconstruction::FillingStrategy fillType;
		};
    }
}

#endif