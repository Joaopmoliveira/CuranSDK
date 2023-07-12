#include "imageprocessing/StaticReconstructor.h"

#include <condition_variable>

namespace curan{
namespace image {

StaticReconstructor::Info::Info(std::array<double,3> in_spacing,std::array<double,3> inorigin, std::array<double,3> size, std::array<std::array<double,3>,3> direction){
	auto inextent = gte::Vector3<double>{size[0]/2.0,size[1]/2.0,size[2]/2.0};

    auto origin = gte::Vector3<double>{inorigin[0],inorigin[1],inorigin[2]};
	std::array<gte::Vector3<double>, 3> alignement;
	alignement[0] = {direction[0][0],direction[0][1],direction[0][2]};
	alignement[1] = {direction[1][0],direction[1][1],direction[1][2]};
	alignement[2] = {direction[2][0],direction[2][1],direction[2][2]};

    auto output_origin = origin
		+ alignement[0] * inextent[0]
		+ alignement[1] * inextent[1]
		+ alignement[2] * inextent[2];

    volumetric_bounding_box = gte::OrientedBox3<double>{output_origin,alignement,inextent};
    spacing[0] = in_spacing[0];
    spacing[1] = in_spacing[1];
    spacing[2] = in_spacing[2];
}

StaticReconstructor::StaticReconstructor(const Info& info) : output_spacing{info.spacing},volumetric_bounding_box{info.volumetric_bounding_box}{
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

	gte::Vector<3, double> origin_gte = volumetric_bounding_box.center
		- volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0]
		- volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1]
		- volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];

    output_type::PointType output_origin;
    output_origin[0] = origin_gte[0];
    output_origin[1] = origin_gte[1];	
    output_origin[2] = origin_gte[2];

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

StaticReconstructor::~StaticReconstructor(){

}

StaticReconstructor& StaticReconstructor::set_interpolation(const curan::image::reconstruction::Interpolation& new_interpolation_strategy){
	std::lock_guard<std::mutex> g{mut};
    interpolation_strategy = new_interpolation_strategy;
    return *(this);
}

StaticReconstructor& StaticReconstructor::set_compound(const curan::image::reconstruction::Compounding& new_compounding_strategy){
	std::lock_guard<std::mutex> g{mut};
    compounding_strategy = new_compounding_strategy;
    return *(this);
}

StaticReconstructor& StaticReconstructor::set_fillstrategy(const curan::image::reconstruction::Compounding& new_compounding_strategy){
	std::lock_guard<std::mutex> g{mut};
    compounding_strategy = new_compounding_strategy;
    return *(this);
}

StaticReconstructor& StaticReconstructor::set_clipping(const Clipping& new_clipping){
	std::lock_guard<std::mutex> g{mut};
    clipping = new_clipping;
    return *(this);
}

StaticReconstructor& StaticReconstructor::add_frame(output_type::Pointer image_pointer)
{
	std::lock_guard<std::mutex> g{mut};
	frame_data.push_back(image_pointer);
    return *(this);
};

StaticReconstructor& StaticReconstructor::add_frames(std::vector<output_type::Pointer>& images_vector)
{
	std::lock_guard<std::mutex> g{mut};
    frame_data.insert(std::end(frame_data), std::begin(images_vector), std::end(images_vector));
    return *(this);
};

StaticReconstructor::output_type::Pointer StaticReconstructor::get_output_pointer(){
	std::lock_guard<std::mutex> g{mut};
    return out_volume;
}

 [[nodiscard]] bool splice_input_extent( std::vector<std::array<int,6>> splitting, const int fullExt[6]){
	std::vector<std::array<int,6>>  splitting_behavior;
	size_t thread_id = 0;
	for(auto& nsplit : splitting){
		int min, max;
  		memcpy( nsplit.data(), fullExt, 6 * sizeof( int ) );
  		int splitAxis = 2; 
  		min = fullExt[4];
  		max = fullExt[5];
  		while ( min == max ){
    		--splitAxis;
    		if ( splitAxis < 0 ){
				//return failure
      			return false;
			}
			min = fullExt[splitAxis * 2];
    		max = fullExt[splitAxis * 2 + 1];
   		}
  		int range = max - min + 1;
  		int valuesPerThread = ( int )std::ceil( range / ( double )splitting.size() );
 		int maxThreadIdUsed = ( int )std::ceil( range / ( double )valuesPerThread ) - 1;
  		if ( thread_id < maxThreadIdUsed ){
    		nsplit[splitAxis * 2] = nsplit[splitAxis * 2] + thread_id * valuesPerThread;
    		nsplit[splitAxis * 2 + 1] = nsplit[splitAxis * 2] + valuesPerThread - 1;
		}
  		if ( thread_id == maxThreadIdUsed )
			nsplit[splitAxis * 2] = nsplit[splitAxis * 2] + thread_id * valuesPerThread;
		++thread_id;
	}

	thread_id = 0;
	std::printf("\n=======\n");
	for(const auto& val : splitting){
		std::printf("for thread: %d extent is: [%d %d] [%d %d] [%d %d]\n",thread_id,val[0],val[1],val[2],val[3],val[4],val[5]);
		++thread_id;
	}
	//return if succedded
  	return true;
}

bool StaticReconstructor::multithreaded_update(std::shared_ptr<utilities::ThreadPool>pool){
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


	std::vector<output_type::Pointer> local_image_copies;
	{
		std::lock_guard<std::mutex> g{mut};
		local_image_copies = std::move(frame_data);
		frame_data = std::vector<output_type::Pointer>();
		if(local_image_copies.size()==0)
			return false;
	}

	for (auto img : local_image_copies) {	
		int inputExtent[6] = { 0, 0, 0, 0, 0, 0};
		double clipRectangleOrigin [2]; // array size 2
		double clipRectangleSize [2]; // array size 2
		if(clipping){
			clipRectangleOrigin[0] = (*clipping).clipRectangleOrigin[0];
			clipRectangleOrigin[1] = (*clipping).clipRectangleOrigin[1];

			clipRectangleSize[0] = (*clipping).clipRectangleSize[0];
			clipRectangleSize[1] = (*clipping).clipRectangleSize[1];

			inputExtent[1] = clipRectangleSize[0];
			inputExtent[3] = clipRectangleSize[1];
		} else {
			auto local_size = img->GetLargestPossibleRegion().GetSize();
			auto local_origin = img->GetOrigin();
			clipRectangleOrigin[0] = local_origin[0];
			clipRectangleOrigin[1] = local_origin[1];
			clipRectangleSize[0] = local_size.GetSize()[0]-1;
			clipRectangleSize[1] = local_size.GetSize()[1]-1;

			inputExtent[1] = clipRectangleSize[0];
			inputExtent[3] = clipRectangleSize[1];
		}	

		paste_slice_info.clipRectangleOrigin = clipRectangleOrigin;
	    paste_slice_info.clipRectangleSize = clipRectangleSize;
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


		// now we need to divide the image between equal patches, for that we copy the IGSIO 
		// block of code 
		std::vector<std::array<int,6>> block_divisions;
		block_divisions.resize(pool->size());
		if(!splice_input_extent(block_divisions,inputExtent))
			throw std::runtime_error("failure to execute slicing of input image");

		std::condition_variable cv;
		std::mutex local_mut;
		std::unique_lock<std::mutex> unique_{local_mut};
		int executed = 0;
		for(const auto& range : block_divisions){
			curan::utilities::Job job;
			job.description = "partial volume reconstruction";
			job.function_to_execute = [&](){
				int this_thread_extent[6];
				std::memcpy(this_thread_extent,range.data(),6*sizeof(int));
				paste_slice_info.inExt = this_thread_extent;
				curan::image::reconstruction::UnoptimizedInsertSlice(&paste_slice_info);
				std::lock_guard<std::mutex> g{local_mut};
				++executed;
				//std::printf("finished patch of work: %d (to do: %d)\n",executed,block_divisions.size());
				cv.notify_one();
			};
			pool->submit(std::move(job));
		}
		//this blocks until all threads have processed their corresponding block that they need to process
		cv.wait(unique_,[&](){ return executed==block_divisions.size();});
	};
	return true;
}

bool StaticReconstructor::update(){
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

	std::vector<output_type::Pointer> local_image_copies;
	{
		std::lock_guard<std::mutex> g{mut};
		local_image_copies = std::move(frame_data);
		frame_data = std::vector<output_type::Pointer>();
		if(local_image_copies.size()==0)
			return false;
	}

	for (auto img : local_image_copies) {	
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
			clipRectangleSize[0] = local_size.GetSize()[0]-1;
			clipRectangleSize[1] = local_size.GetSize()[1]-1;

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
	return true;
}

}
}