#ifndef CURAN_INTEGRATED_RECONSTRUCTOR_HEADER_FILE_
#define CURAN_INTEGRATED_RECONSTRUCTOR_HEADER_FILE_

#include "ImageProcessingDefinitions.h"
#include "KernelDescriptor.h"
#include "Mathematics/ConvexHull3.h"
#include "Mathematics/ArbitraryPrecision.h"
#include "Mathematics/MinimumVolumeBox3.h"
#include "VolumeAlgorithms.h"
#include <optional>
#include <mutex>
#include "utils/TheadPool.h"
#include <vsg/all.h>
#include <vsgXchange/all.h>
#include "rendering/Renderable.h"

namespace curan{
    namespace image {

        struct Clipping{
	        std::array<double, 2> clipRectangleOrigin = { 0.0,0.0 }; 
	        std::array<double, 2> clipRectangleSize = { 0.0,0.0 };
        };

        struct IntegratedReconstructor : public vsg::Inherit<renderable::Renderable, IntegratedReconstructor>{
        public:
            static constexpr size_t Dimension = 3;
            using input_pixel_type = unsigned char;
            using input_type = itk::Image<input_pixel_type, Dimension>;
            using output_pixel_type = float;
	        using output_type = itk::Image<output_pixel_type, Dimension>;
            using accumulator_type = itk::Image<unsigned short, Dimension>;
	        using resampler_output = itk::ResampleImageFilter<output_type, output_type>;
	        using resampler_accumulator = itk::ResampleImageFilter<accumulator_type, accumulator_type>;
        private:
	        gte::OrientedBox3<double> volumetric_bounding_box;
	        curan::image::reconstruction::Interpolation interpolation_strategy = curan::image::reconstruction::Interpolation::NEAREST_NEIGHBOR_INTERPOLATION;
	        curan::image::reconstruction::Compounding compounding_strategy = curan::image::reconstruction::Compounding::LATEST_COMPOUNDING_MODE;

            std::optional<Clipping> clipping;
	        std::vector<input_type::Pointer> frame_data;

	        output_type::SpacingType output_spacing;
	        itk::Point<double,3> origin;
	        accumulator_type::Pointer acummulation_buffer;
	        output_type::Pointer out_volume; //this is a non-owning copy of the texture Data bellow, it is only used for simplicity

	        std::vector<curan::image::reconstruction::KernelDescriptor> kernels;
	        curan::image::reconstruction::FillingStrategy fillType;
			std::mutex mut;
        public:

            struct Info{
                std::optional<std::string> identifier;
                gte::OrientedBox3<double> volumetric_bounding_box;
                output_type::SpacingType spacing;
                Info(std::array<double,3> spacing,std::array<double,3> origin, std::array<double,3> size, std::array<std::array<double,3>,3> direction);
            };
            vsg::ref_ptr<vsg::floatArray3D> textureData;
            output_type::SizeType output_size;
            itk::Image<curan::image::char_pixel_type, 3U> *  volume_temp;
            

	    static constexpr int INPUT_COMPONENTS = 1;

        static vsg::ref_ptr<renderable::Renderable> make(Info& info);

	    IntegratedReconstructor(const Info& info);

	    ~IntegratedReconstructor();

        IntegratedReconstructor& set_interpolation(const curan::image::reconstruction::Interpolation& new_interpolation_strategy);

        IntegratedReconstructor& set_compound(const curan::image::reconstruction::Compounding& new_compounding_strategy);

        IntegratedReconstructor& set_fillstrategy(const curan::image::reconstruction::FillingStrategy& new_filling_strategy);

        IntegratedReconstructor& set_clipping(const Clipping& new_clipping);

        IntegratedReconstructor& add_frame(input_type::Pointer image_pointer);

        IntegratedReconstructor& add_frames(std::vector<input_type::Pointer>& images_vector);

        vsg::ref_ptr<vsg::floatArray3D> get_texture_data();
   

        itk::Size<3U> get_output_size();

		void add_kernel_descritor(curan::image::reconstruction::KernelDescriptor descriptor);

		void fill_holes();

	    bool update();

        inline void reset(){
            std::lock_guard<std::mutex> g{mut};
            for(auto pix = textureData->begin() ; pix != textureData->end(); ++pix)
                *pix = 0.0;
        } 

		bool multithreaded_update(std::shared_ptr<utilities::ThreadPool>pool);
};


}
}

#endif