#include "imageprocessing/VolumeAlgorithms.h"
#include <optional>

struct Clipping{
	std::array<double, 2> clipRectangleOrigin = { 0.0,0.0 }; 
	std::array<double, 2> clipRectangleSize = { 0.0,0.0 };
};

class StaticReconstructor{
public:
    static constexpr size_t Dimension = 3;
	using output_type = itk::Image<unsigned char, Dimension>;
    using accumulator_type = itk::Image<unsigned short, Dimension>;
	using resampler_output = itk::ResampleImageFilter<output_type, output_type>;
	using resampler_accumulator = itk::ResampleImageFilter<accumulator_type, accumulator_type>;
private:
	std::array<std::array<double,3>,3> axis;
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
public:

    struct Info{
        std::array<std::array<double,3>,3> axis;
        itk::Point<double,3> origin;
        output_type::SpacingType spacing;
    };

	// With this code we always assume 
	// that the images are greyscale
	// therefore there is a single value
	// to manipulate
	static constexpr int INPUT_COMPONENTS = 1;

	StaticReconstructor(const Info& info) : output_spacing{info.spacing},axis{info.axis},origin{info.origin}{
		out_volume = output_type::New();
		acummulation_buffer = accumulator_type::New();
    }

	~StaticReconstructor(){

    }

    StaticReconstructor& set_interpolation(const curan::image::reconstruction::Interpolation& new_interpolation_strategy){
        interpolation_strategy = new_interpolation_strategy;
        return *(this);
    }

    StaticReconstructor& set_compound(const curan::image::reconstruction::Compounding& new_compounding_strategy){
        compounding_strategy = new_compounding_strategy;
        return *(this);
    }

    StaticReconstructor& set_fillstrategy(const curan::image::reconstruction::Compounding& new_compounding_strategy){
        compounding_strategy = new_compounding_strategy;
        return *(this);
    }

    StaticReconstructor& set_clipping(const Clipping& new_clipping){
        clipping = new_clipping;
        return *(this);
    }

    StaticReconstructor& add_frame(output_type::Pointer image_pointer)
    {
	    frame_data.push_back(image_pointer);
        return *(this);
    };

    StaticReconstructor& add_frames(std::vector<output_type::Pointer>& images_vector)
    {
	    frame_data.insert(std::end(frame_data), std::begin(images_vector), std::end(images_vector));
        return *(this);
    };

	StaticReconstructor& update(){
		Eigen::Matrix4d ref_to_output_origin;
		ref_to_output_origin(0, 0) = axis[0][0];
		ref_to_output_origin(1, 0) = axis[0][1];
		ref_to_output_origin(2, 0) = axis[0][2];
		ref_to_output_origin(3, 0) = 0.0;

		ref_to_output_origin(0, 1) = axis[1][0];
		ref_to_output_origin(1, 1) = axis[1][1];
		ref_to_output_origin(2, 1) = axis[1][2];
		ref_to_output_origin(3, 1) = 0.0;

		ref_to_output_origin(0, 2) = axis[2][0];
		ref_to_output_origin(1, 2) = axis[2][1];
		ref_to_output_origin(2, 2) = axis[2][2];
		ref_to_output_origin(3, 2) = 0.0;

		ref_to_output_origin(0, 3) = origin[0];
		ref_to_output_origin(1, 3) = origin[1];
		ref_to_output_origin(2, 3) = origin[2];
		ref_to_output_origin(3, 3) = 1.0;

		Eigen::Matrix4d output_to_ref = ref_to_output_origin.inverse();

		unsigned int accOverflow = 20;

		curan::image::reconstruction::PasteSliceIntoVolumeInsertSliceParams paste_slice_info;
		paste_slice_info.outData = out_volume;
		paste_slice_info.outPtr = out_volume->GetBufferPointer();
		paste_slice_info.accPtr = acummulation_buffer->GetBufferPointer();
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
			int inputFrameExtentForCurrentThread[6] = { 0, 0, 0, 0, 0, 0 };
			double clipRectangleOrigin [2]; // array size 2
			double clipRectangleSize [2]; // array size 2
			if(clipping){
				clipRectangleOrigin;
				clipRectangleSize;
				inputFrameExtentForCurrentThread;
			} else {
				clipRectangleOrigin;
				clipRectangleSize;
				inputFrameExtentForCurrentThread;
			}	
			paste_slice_info.clipRectangleOrigin = clipRectangleOrigin;
			paste_slice_info.clipRectangleSize = clipRectangleSize;
			paste_slice_info.inExt = inputFrameExtentForCurrentThread;
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

			paste_slice_info.inData = img;
			paste_slice_info.inPtr = img->GetBufferPointer();
			paste_slice_info.matrix = output_to_origin;

			curan::image::reconstruction::UnoptimizedInsertSlice(&paste_slice_info);
		};

		frame_data.clear();
	}
};

int main(){

};