#include "imageprocessing/VolumeAlgorithms.h"
#include <optional>
#include "Mathematics/ConvexHull3.h"
#include "Mathematics/ArbitraryPrecision.h"
#include "Mathematics/MinimumVolumeBox3.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"

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
public:

    struct Info{
        gte::OrientedBox3<double> volumetric_bounding_box;
        output_type::SpacingType spacing;
    };

	// With this code we always assume 
	// that the images are greyscale
	// therefore there is a single value
	// to manipulate
	static constexpr int INPUT_COMPONENTS = 1;

	StaticReconstructor(const Info& info) : output_spacing{info.spacing},volumetric_bounding_box{info.volumetric_bounding_box}{
		output_type::IndexType output_start;
		output_start[0] = 0;
		output_start[1] = 0;
		output_start[2] = 0;

		output_type::DirectionType output_directorion;
		output_directorion[0][0] = volumetric_bounding_box.axis[0][0];
		output_directorion[1][0] = volumetric_bounding_box.axis[1][0];
		output_directorion[2][0] = volumetric_bounding_box.axis[2][0];

		output_directorion[0][1] = volumetric_bounding_box.axis[0][1];
		output_directorion[1][1] = volumetric_bounding_box.axis[1][1];
		output_directorion[2][1] = volumetric_bounding_box.axis[2][1];

		output_directorion[0][2] = volumetric_bounding_box.axis[0][2];
		output_directorion[1][2] = volumetric_bounding_box.axis[1][2];
		output_directorion[2][2] = volumetric_bounding_box.axis[2][2];

		output_type::SizeType output_size;
		output_size[0] = std::ceil(volumetric_bounding_box.extent[0] * 2 / output_spacing[0]);
		output_size[1] = std::ceil(volumetric_bounding_box.extent[1] * 2 / output_spacing[1]);
		output_size[2] = std::ceil(volumetric_bounding_box.extent[2] * 2 / output_spacing[2]);

		output_type::PointType output_origin;
		output_origin[0] = origin[0];
		output_origin[1] = origin[1];	
		output_origin[2] = origin[2];

		output_type::RegionType output_region;
		output_region.SetSize(output_size);
		output_region.SetIndex(output_start);

		out_volume = output_type::New();
		out_volume->SetRegions(output_region);
		out_volume->SetOrigin(output_origin);
		out_volume->SetSpacing(output_spacing);
		out_volume->SetDirection(output_directorion);
		out_volume->Allocate(true);

		acummulation_buffer = accumulator_type::New();
		acummulation_buffer->SetRegions(output_region);
		acummulation_buffer->SetOrigin(output_origin);
		acummulation_buffer->SetSpacing(output_spacing);
		acummulation_buffer->SetDirection(output_directorion);
		acummulation_buffer->Allocate(true);
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

	output_type::Pointer get_output_pointer(){
		return out_volume;
	}

	StaticReconstructor& update(){
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
				clipRectangleOrigin[0] = (*clipping).clipRectangleOrigin[0];
				clipRectangleOrigin[1] = (*clipping).clipRectangleOrigin[1];

				clipRectangleSize[0] = (*clipping).clipRectangleSize[0];
				clipRectangleSize[1] = (*clipping).clipRectangleSize[1];

				inputFrameExtentForCurrentThread[1] = clipRectangleSize[0];
				inputFrameExtentForCurrentThread[3] = clipRectangleSize[1];
			} else {
				auto local_size = img->GetLargestPossibleRegion().GetSize();
				auto local_origin = img->GetOrigin();
				clipRectangleOrigin[0] = local_origin[0];
				clipRectangleOrigin[1] = local_origin[1];
				clipRectangleSize[0] = local_size.GetSize()[0];
				clipRectangleSize[1] = local_size.GetSize()[1];

				inputFrameExtentForCurrentThread[1] = clipRectangleSize[0];
				inputFrameExtentForCurrentThread[3] = clipRectangleSize[1];
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
		return *(this);
	}


};

constexpr long width = 50;
constexpr long height = 50;
constexpr double offset = 1;
float spacing[3] = {0.02 , 0.02 , 1};
float final_spacing [3] = {1 ,1, 1};

void create_array_of_linear_images_in_x_direction(std::vector<StaticReconstructor::output_type::Pointer>& desired_images){
    // the volume shoud be a box with spacing 1.0 mm in all directions with a size of 2 mm in each side 
    // to fill the volume we create an image with two images 

    itk::Matrix<double> image_orientation;
	itk::Point<double> image_origin;

	image_orientation[0][0] = 1.0;
	image_orientation[1][0] = 0.0;
	image_orientation[2][0] = 0.0;

	image_orientation[0][1] = 0.0;
	image_orientation[1][1] = 1.0;
	image_orientation[2][1] = 0.0;

	image_orientation[0][2] = 0.0;
	image_orientation[1][2] = 0.0;
	image_orientation[2][2] = 1.0;

	image_origin[0] = 0.0;
	image_origin[1] = 0.0;
	image_origin[2] = 0.0;

    curan::image::char_pixel_type pixel[width*height];
    for(size_t y = 0; y < height ; ++y)
        for(size_t x = 0; x < width ; ++x)
            pixel[x+y*height] = (int) std::sqrt(y*y+x*x);
	
	
    for(int z = 0; z < width ; ++z){
        StaticReconstructor::output_type::Pointer image = StaticReconstructor::output_type::New();
        StaticReconstructor::output_type::IndexType start;
        start[0] = 0; // first index on X
        start[1] = 0; // first index on Y
        start[2] = 0; // first index on Z

        StaticReconstructor::output_type::SizeType size;
        size[0] = width; // size along X
        size[1] = height; // size along Y
        size[2] = 1; // size along Z

        StaticReconstructor::output_type::RegionType region_1;
        region_1.SetSize(size);
        region_1.SetIndex(start);

        image->SetRegions(region_1);
        image->SetDirection(image_orientation);
        image->SetOrigin(image_origin);
        image->SetSpacing(spacing);
        image->Allocate();

        image_origin[0] = 0.0;
	    image_origin[1] = 0.0;
	    image_origin[2] = std::sin((1/50.0)*z);

        auto pointer = image->GetBufferPointer();
        std::memcpy(pointer,pixel,sizeof(curan::image::char_pixel_type)*width*height);
        desired_images.push_back(image);
    }
}

void updateBaseTexture3D(vsg::floatArray3D& image, StaticReconstructor::output_type::Pointer image_to_render)
{
    using OutputPixelType = float;
    using InputImageType = itk::Image<unsigned char, 3>;
    using OutputImageType = itk::Image<OutputPixelType, 3>;
    using FilterType = itk::CastImageFilter<InputImageType, OutputImageType>;
    auto filter = FilterType::New();
    filter->SetInput(image_to_render);

    using RescaleType = itk::RescaleIntensityImageFilter<OutputImageType, OutputImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(filter->GetOutput());
    rescale->SetOutputMinimum(0.0);
    rescale->SetOutputMaximum(1.0);

    try{
        rescale->Update();
    } catch (const itk::ExceptionObject& e) {
        std::cerr << "Error: " << e << std::endl;
        throw std::runtime_error("error");
    }

    OutputImageType::Pointer out = rescale->GetOutput();

    using IteratorType = itk::ImageRegionIteratorWithIndex<OutputImageType>;
    IteratorType outputIt(out, out->GetRequestedRegion());
    for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt){
        StaticReconstructor::output_type::IndexType idx = outputIt.GetIndex();
		std::printf("%d,%d,%d,val: %f\n",idx[0],idx[1],idx[2],outputIt.Get());
        image.set(idx[0], idx[1], idx[2], outputIt.Get());
    }
}

int main(){
	std::vector<StaticReconstructor::output_type::Pointer> image_array;
	create_array_of_linear_images_in_x_direction(image_array);
	size_t counter = 0;
	for(auto& img : image_array){
		auto size = img->GetLargestPossibleRegion().GetSize();
		int height = size[1];
		int width = size[0];

		StaticReconstructor::output_type::PointType origin_position;
		StaticReconstructor::output_type::IndexType origin_pixel = { 0,0,0 };
		img->TransformIndexToPhysicalPoint(origin_pixel, origin_position);

		auto vert1 = gte::Vector3<double>({ origin_position[0], origin_position[1], origin_position[2] });
		std::printf("image %d - \n\tvertex 1 : ( %f %f %f )\n",counter,vert1[0],vert1[1],vert1[2]);

		StaticReconstructor::output_type::IndexType origin_along_width = { width - 1,0,0 };
		StaticReconstructor::output_type::PointType origin_along_width_position;
		img->TransformIndexToPhysicalPoint(origin_along_width, origin_along_width_position);

		auto vert2 = gte::Vector3<double>({ origin_along_width_position[0], origin_along_width_position[1], origin_along_width_position[2] });
		std::printf("\tvertex 2 : ( %f %f %f )\n",vert2[0],vert2[1],vert2[2]);

		StaticReconstructor::output_type::IndexType origin_along_width_and_height = { width - 1,height - 1,0 };
		StaticReconstructor::output_type::PointType origin_along_width_and_height_position;
		img->TransformIndexToPhysicalPoint(origin_along_width_and_height, origin_along_width_and_height_position);

		auto vert3 = gte::Vector3<double>({ origin_along_width_and_height_position[0], origin_along_width_and_height_position[1], origin_along_width_and_height_position[2] });
		std::printf("\tvertex 3 : ( %f %f %f )\n",vert3[0],vert3[1],vert3[2]);

		StaticReconstructor::output_type::IndexType origin_along_height = { 0,height - 1,0 };
		StaticReconstructor::output_type::PointType origin_along_height_position;
		img->TransformIndexToPhysicalPoint(origin_along_height, origin_along_height_position);

		auto vert4 = gte::Vector3<double>({ origin_along_height_position[0], origin_along_height_position[1], origin_along_height_position[2] });
		std::printf("\tvertex 4 : ( %f %f %f )\n",vert4[0],vert4[1],vert4[2]);
		++counter;
	}
	StaticReconstructor::Info recon_info;
	recon_info.spacing[0] = 0.02;
	recon_info.spacing[1] = 0.02;
	recon_info.spacing[2] = 0.02;
	auto origin = gte::Vector3<double>{0.0,0.0,0.0};
	std::array<gte::Vector3<double>, 3> alignement;
	alignement[0] = {1.0,0.0,0.0};
	alignement[1] = {0.0,1.0,0.0};
	alignement[2] = {0.0,0.0,1.0};
	auto inextent = gte::Vector3<double>{1.0,1.0,1.0};

	gte::OrientedBox3<double> box {origin,alignement,inextent};
	recon_info.volumetric_bounding_box = box;
	StaticReconstructor reconstructor{recon_info};
	reconstructor.set_compound(curan::image::reconstruction::Compounding::LATEST_COMPOUNDING_MODE).add_frames(image_array);
	reconstructor.update();

	auto buffer = reconstructor.get_output_pointer();

	curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "myviewer";
    curan::renderable::Window::WindowSize size{1000, 800};
    info.window_size = size;
    curan::renderable::Window window{info};

   	auto sizeimage = buffer->GetLargestPossibleRegion().GetSize();
    std::printf("size : ");
    std::cout << sizeimage << std::endl;
    
    curan::renderable::Volume::Info volumeinfo;
    volumeinfo.width = sizeimage.GetSize()[0]; 
    volumeinfo.height = sizeimage.GetSize()[1];
    volumeinfo.depth = sizeimage.GetSize()[2];
    volumeinfo.spacing_x = final_spacing[0];
    volumeinfo.spacing_y = final_spacing[1];
    volumeinfo.spacing_z = final_spacing[2];
    
    auto volume = curan::renderable::Volume::make(volumeinfo);
	auto casted_volume = volume->cast<curan::renderable::Volume>();
    auto updater = [buffer](vsg::floatArray3D& image){
        updateBaseTexture3D(image, buffer);
    };
    casted_volume->update_volume(updater);
    window << volume;
    
    window.run();

    window.transverse_identifiers(
            [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable >>
                   &map) {
                for (auto &p : map){
                    std::cout << "Object contained: " << p.first << '\n';
                }
    });
	return 0;
};