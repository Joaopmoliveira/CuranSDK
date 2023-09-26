#ifndef CURAN_BOUNDING_BOX_FOR_RECONSTRUCTION_HEADER_FILE_
#define CURAN_BOUNDING_BOX_FOR_RECONSTRUCTION_HEADER_FILE_

#include "ImageProcessingDefinitions.h"
#include "Mathematics/ConvexHull3.h"
#include "Mathematics/ArbitraryPrecision.h"
#include "Mathematics/MinimumVolumeBox3.h"
#include "VolumeAlgorithms.h"

namespace curan{
    namespace image{

		class BoundingBox4Reconstruction
		{
		public:
			using output_type = itk::Image<char_pixel_type, Dimension3D>;
			using array_type = std::array<gte::Vector3<double>, 8>;

			BoundingBox4Reconstruction();
			~BoundingBox4Reconstruction();

			void update();

			void add_frames(std::vector<output_type::Pointer>&);

			void add_frame(output_type::Pointer);

			inline gte::OrientedBox3<double> get_final_volume_vertices(){
				return volumetric_bounding_box;
			}

			gte::OrientedBox3<double> volumetric_bounding_box;
		private:
			std::vector<output_type::Pointer> frame_data;
			std::vector<gte::Vector3<double>> current_vertices;
		};
    }
}

#endif