#ifndef CURAN_BOUNDING_BOX_FOR_RECONSTRUCTION_HEADER_FILE_
#define CURAN_BOUNDING_BOX_FOR_RECONSTRUCTION_HEADER_FILE_

#include "ImageProcessingDefinitions.h"
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
		class BoundingBox4Reconstruction
		{
		public:
			using output_type = itk::Image<char_pixel_type, Dimension3D>;
			using array_type = std::array<gte::Vector3<double>, 8>;

			BoundingBox4Reconstruction();
			~BoundingBox4Reconstruction();

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

			void add_frames(std::vector<output_type::Pointer>&);
			/*
			When one has all the frames which will be used to
			for the volumetric reconstruction, they can provide all 
			images at once, thus removing needless time spent submitting 
			the images.
			*/

			/* Add just one frame at each time */
			void add_frame(output_type::Pointer);

			/*
			Obtain the pointer to the 3D itk volume reconstructed
			and filled under the hood with this class.
			*/
			inline gte::OrientedBox3<double> get_final_volume_vertices(){
				return volumetric_bounding_box;
			}


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

			std::vector<output_type::Pointer> frame_data;

			bool volumes_initiated = false;
			gte::OrientedBox3<double> volumetric_bounding_box;

		};
    }
}

#endif