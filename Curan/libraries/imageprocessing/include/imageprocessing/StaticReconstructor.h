#ifndef CURAN_STATIC_RECONSTRUCTOR_HEADER_FILE_
#define CURAN_STATIC_RECONSTRUCTOR_HEADER_FILE_

#include "ImageProcessingDefinitions.h"
#include "KernelDescriptor.h"
#include "Mathematics/ConvexHull3.h"
#include "Mathematics/ArbitraryPrecision.h"
#include "Mathematics/MinimumVolumeBox3.h"
#include "VolumeAlgorithms.h"
#include <optional>
#include <mutex>
#include "utils/TheadPool.h"

namespace curan{
    namespace image {
        struct Clipping{
	        std::array<double, 2> clipRectangleOrigin = { 0.0,0.0 }; 
	        std::array<double, 2> clipRectangleSize = { 0.0,0.0 };
        };

		[[nodiscard]] bool splice_input_extent( std::vector<std::array<int,6>>& splitting, const int fullExt[6]);

        class StaticReconstructor{
        public:
            static constexpr size_t Dimension = 3;
	        using output_type = itk::Image<unsigned char, Dimension>;
            using accumulator_type = itk::Image<unsigned short, Dimension>;
	        using resampler_output = itk::ResampleImageFilter<output_type, output_type>;
	        using resampler_accumulator = itk::ResampleImageFilter<accumulator_type, accumulator_type>;
        private:
	        gte::OrientedBox3<double> volumetric_bounding_box;
	        curan::image::reconstruction::Interpolation interpolation_strategy = curan::image::reconstruction::Interpolation::NEAREST_NEIGHBOR_INTERPOLATION;
	        curan::image::reconstruction::Compounding compounding_strategy = curan::image::reconstruction::Compounding::LATEST_COMPOUNDING_MODE;

            std::optional<Clipping> clipping;
	        std::vector<output_type::Pointer> frame_data;

	        output_type::SpacingType output_spacing;
	        itk::Point<double,3> origin;
	        output_type::Pointer out_volume;
	        accumulator_type::Pointer acummulation_buffer;

	        std::vector<curan::image::reconstruction::KernelDescriptor> kernels;
	        curan::image::reconstruction::FillingStrategy fillType;
			std::mutex mut;
        public:

            struct Info{
                gte::OrientedBox3<double> volumetric_bounding_box;
                output_type::SpacingType spacing;
                Info(std::array<double,3> spacing,std::array<double,3> origin, std::array<double,3> size, std::array<std::array<double,3>,3> direction);
            };

	    // With this code we always assume 
	    // that the images are greyscale
	    // therefore there is a single value
	    // to manipulate
	    static constexpr int INPUT_COMPONENTS = 1;

	    StaticReconstructor(const Info& info);

	    ~StaticReconstructor();

        StaticReconstructor& set_interpolation(const curan::image::reconstruction::Interpolation& new_interpolation_strategy);

        StaticReconstructor& set_compound(const curan::image::reconstruction::Compounding& new_compounding_strategy);

        StaticReconstructor& set_fillstrategy(const curan::image::reconstruction::Compounding& new_compounding_strategy);

        StaticReconstructor& set_clipping(const Clipping& new_clipping);

        StaticReconstructor& add_frame(output_type::Pointer image_pointer);

        StaticReconstructor& add_frames(std::vector<output_type::Pointer>& images_vector);

	    output_type::Pointer get_output_pointer();

	    bool update();

		bool multithreaded_update(std::shared_ptr<utilities::ThreadPool>pool);
};


}
}

#endif