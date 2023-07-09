#include "imageprocessing/VolumeReconstructor.h"
#include "imageprocessing/VolumeAlgorithms.h"
#include "utils/Logger.h"

namespace curan {
namespace image {

VolumeReconstructor::VolumeReconstructor()
{
	if (out_volume.IsNull())
	{
		out_volume = InternalImageType::New();
	}

	if (acummulation_buffer.IsNull())
	{
		acummulation_buffer = itk::Image<short_pixel_type, Dimension3D>::New();
	}

	output_spacing[0] = 1.0;
	output_spacing[1] = 1.0;
	output_spacing[2] = 1.0;

}

VolumeReconstructor::~VolumeReconstructor()
{
}

void VolumeReconstructor::update()
{
	std::vector<gte::Vector3<double>> vertices;

	// We multiply by four because each 
	// image has four courners and we 
	// add eight because we have the 
	// current eight corners in memory 
	// which represent the minimum bounding 
	// box containing all the frames 
	// already added to memory
	vertices.resize(frame_data.size() * 4 + 8);

	int increment = 0;
	int counter = 0 ;
	for (auto& img : frame_data)
	{
		auto size = img->GetLargestPossibleRegion().GetSize();
		int height = size[1];
		int width = size[0];

		output_type::PointType origin_position;
		output_type::IndexType origin_pixel = { 0,0,0 };
		img->TransformIndexToPhysicalPoint(origin_pixel, origin_position);

		vertices[increment] = gte::Vector3<double>({ origin_position[0], origin_position[1], origin_position[2] });
		//std::printf("image %d - \n\tvertex 1 : ( %f %f %f )\n",counter,vertices[increment][0],vertices[increment][1],vertices[increment][2]);

		output_type::IndexType origin_along_width = { width - 1,0,0 };
		output_type::PointType origin_along_width_position;
		img->TransformIndexToPhysicalPoint(origin_along_width, origin_along_width_position);

		vertices[increment + 1] = gte::Vector3<double>({ origin_along_width_position[0], origin_along_width_position[1], origin_along_width_position[2] });
		//std::printf("\tvertex 2 : ( %f %f %f )\n",counter,vertices[increment+1][0],vertices[increment+1][1],vertices[increment+1][2]);

		output_type::IndexType origin_along_width_and_height = { width - 1,height - 1,0 };
		output_type::PointType origin_along_width_and_height_position;
		img->TransformIndexToPhysicalPoint(origin_along_width_and_height, origin_along_width_and_height_position);

		vertices[increment + 2] = gte::Vector3<double>({ origin_along_width_and_height_position[0], origin_along_width_and_height_position[1], origin_along_width_and_height_position[2] });
		//std::printf("\tvertex 3 : ( %f %f %f )\n",counter,vertices[increment+2][0],vertices[increment+2][1],vertices[increment+2][2]);

		output_type::IndexType origin_along_height = { 0,height - 1,0 };
		output_type::PointType origin_along_height_position;
		img->TransformIndexToPhysicalPoint(origin_along_height, origin_along_height_position);

		vertices[increment + 3] = gte::Vector3<double>({ origin_along_height_position[0], origin_along_height_position[1], origin_along_height_position[2] });
		//std::printf("\tvertex 4 : ( %f %f %f )\n",counter,vertices[increment+3][0],vertices[increment+3][1],vertices[increment+3][2]);
		increment += 4;
		++counter;
	};

	// if the bounding box is still unitialized 
	// then we need to run the algorithm without 
	// the vertices stored in current_corners
	if (!volumes_initiated)
	{
		gte::MinimumVolumeBox3<double, true> bounding_box(0);

		double volume = 0.0;
		bounding_box(frame_data.size() * 4, vertices.data(), 4, volumetric_bounding_box, volume);
	} else {
		std::array<gte::Vector3<double>, 8> current_corners;
		current_corners[0] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
		current_corners[1] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
		current_corners[2] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
		current_corners[3] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
		current_corners[4] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
		current_corners[5] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
		current_corners[6] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
		current_corners[7] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];

		vertices.insert(vertices.begin() + frame_data.size() * 4, std::begin(current_corners), std::begin(current_corners));
		gte::MinimumVolumeBox3<double, true> bounding_box(0);

		double volume = 0.0;
		bounding_box(vertices.size(), vertices.data(), 4, volumetric_bounding_box, volume);
	}

	//since we might have enlarged the size of the output volume with the new pixels added we need to update the current volume.
	update_internal_buffers();

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
	int inputFrameExtentForCurrentThread[6] = { 0, (int)std::ceil(clipRectangleSize[0]) - 1, 0, (int)std::ceil(clipRectangleSize[1]) - 1, 0, 0 };

	reconstruction::PasteSliceIntoVolumeInsertSliceParams paste_slice_info;
	paste_slice_info.outData = out_volume;
	paste_slice_info.outPtr = out_volume->GetBufferPointer();
	paste_slice_info.accPtr = acummulation_buffer->GetBufferPointer();
	paste_slice_info.clipRectangleOrigin = &clipRectangleOrigin[0];
	paste_slice_info.clipRectangleSize = &clipRectangleSize[0];
	paste_slice_info.inExt = inputFrameExtentForCurrentThread;
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

		UnoptimizedInsertSlice(&paste_slice_info);

	};

	frame_data.clear();
};

void VolumeReconstructor::add_frame(output_type::Pointer image_pointer)
{
	frame_data.push_back(image_pointer);
};

void VolumeReconstructor::add_frames(std::vector<output_type::Pointer>& images_vector)
{
	frame_data.insert(std::end(frame_data), std::begin(images_vector), std::end(images_vector));
};

void VolumeReconstructor::set_output_spacing(output_type::SpacingType in_output_spacing)
{
	output_spacing = in_output_spacing;
};

void VolumeReconstructor::set_clipping_bounds(std::array<double, 2> inclipRectangleOrigin, std::array<double, 2> inclipRectangleSize)
{
	clipRectangleOrigin = inclipRectangleOrigin;
	clipRectangleSize = inclipRectangleSize;
}

void VolumeReconstructor::get_output_pointer(output_type::Pointer& pointer_to_be_changed){
	pointer_to_be_changed = out_volume;
};

void VolumeReconstructor::set_fill_strategy(reconstruction::FillingStrategy strategy)
{
	fillType = strategy;
};

void VolumeReconstructor::add_kernel_descritor(reconstruction::KernelDescriptor descriptor)
{
	if (fillType == descriptor.fillType) {
		descriptor.Allocate();
		kernels.push_back(descriptor);
	}
};

void VolumeReconstructor::fill_holes()
{
	char_pixel_type* inVolPtr = out_volume->GetBufferPointer();
	short_pixel_type* accPtr = acummulation_buffer->GetBufferPointer();

	//we need to create the output volume where the 
	//voxels will be placed after the filling procedure is over
	itk::ImageDuplicator<InternalImageType>::Pointer duplicator = itk::ImageDuplicator<InternalImageType>::New();
	duplicator->SetInputImage(out_volume);
	duplicator->Update();
	InternalImageType::Pointer filled_volume = duplicator->GetOutput();
	char_pixel_type* outPtr = filled_volume->GetBufferPointer();

	auto outdata_ROI = out_volume->GetLargestPossibleRegion();
	auto size_out = outdata_ROI.GetSize();
	auto start_out = outdata_ROI.GetIndex();

	uint64_t outExt[6];
	outExt[0] = start_out[0];
	outExt[1] = size_out[0] - 1;

	outExt[2] = start_out[1];
	outExt[3] = size_out[1] - 1;

	outExt[4] = start_out[2];
	outExt[5] = size_out[2] - 1;

	// get increments for volume and for accumulation buffer
	uint64_t byteIncVol[3] = { 0 }; //x,y,z

	int idx;
	uint64_t incr = reconstruction::INPUT_COMPONENTS;

	for (idx = 0; idx < 3; ++idx)
	{
		byteIncVol[idx] = incr;
		incr *= (outExt[idx * 2 + 1] - outExt[idx * 2] + 1);
	}

	// this will store the position of the pixel being looked at currently
	uint64_t currentPos[3]; //x,y,z

	uint64_t numVolumeComponents = reconstruction::INPUT_COMPONENTS;

	// Set interpolation method - nearest neighbor or trilinear
	bool (*apply)(char_pixel_type * inputData,
		unsigned short* accData,
		uint64_t * inputOffsets,
		uint64_t * bounds,
		uint64_t * wholeExtent,
		uint64_t * thisPixel,
		char_pixel_type & returnVal,
		const reconstruction::KernelDescriptor * descriptor) = NULL;

	switch (fillType)
	{
	case reconstruction::FillingStrategy::GAUSSIAN:
		apply = &reconstruction::ApplyGaussian;
		break;
	case reconstruction::FillingStrategy::GAUSSIAN_ACCUMULATION:
		apply = &reconstruction::ApplyGaussianAccumulation;
		break;
	case reconstruction::FillingStrategy::DISTANCE_WEIGHT_INVERSE:
		apply = &reconstruction::ApplyDistanceWeightInverse;
		break;
	case reconstruction::FillingStrategy::NEAREST_NEIGHBOR:
		apply = &reconstruction::ApplyNearestNeighbor;
		break;
	case reconstruction::FillingStrategy::STICK:
		apply = &reconstruction::ApplySticks;
		break;
	default:
	{
		std::string s = "Unknown interpolation mode: " + std::to_string(fillType);
		utilities::cout << s;
		return;
	}
	}

	// iterate through each voxel. When the accumulation buffer is 0, fill that hole, and continue.
	for (currentPos[2] = outExt[4]; currentPos[2] <= outExt[5]; currentPos[2]++)
	{
		for (currentPos[1] = outExt[2]; currentPos[1] <= outExt[3]; currentPos[1]++)
		{
			for (currentPos[0] = outExt[0]; currentPos[0] <= outExt[1]; currentPos[0]++)
			{
				// accumulator index should not depend on which individual component is being interpolated
				int accIndex = (currentPos[0] * byteIncVol[0]) + (currentPos[1] * byteIncVol[1]) + (currentPos[2] * byteIncVol[2]);
				if (accPtr[accIndex] == 0) // if not hit by accumulation during vtkIGSIOPasteSliceIntoVolume
				{
					bool result(false);
					for (const auto kernel : kernels)
					{
						result = apply(inVolPtr, accPtr, byteIncVol, outExt, outExt, currentPos, outPtr[accIndex], &kernel);
						if (result) {
							break;
						} // end checking interpolation success
					}
				} else // if hit, just use the apparent value
				{
					int volCompIndex = (currentPos[0] * byteIncVol[0]) + (currentPos[1] * byteIncVol[1]) + (currentPos[2] * byteIncVol[2]);
					outPtr[volCompIndex] = inVolPtr[volCompIndex];
				} // end accumulation check
			} // end x loop
		} // end y loop
	} // end z loop

	out_volume = filled_volume;
};


bool VolumeReconstructor::update_internal_buffers()
{
	static gte::OrientedBox3<double> previous_box;

	if (previous_box != volumetric_bounding_box)
	{
		previous_box = volumetric_bounding_box;

		if (!volumes_initiated) {

			output_type::IndexType output_start;
			output_start[0] = 0;
			output_start[1] = 0;
			output_start[2] = 0;

			output_type::SizeType output_size;
			output_size[0] = std::ceil(volumetric_bounding_box.extent[0] * 2 / output_spacing[0]);
			output_size[1] = std::ceil(volumetric_bounding_box.extent[1] * 2 / output_spacing[1]);
			output_size[2] = std::ceil(volumetric_bounding_box.extent[2] * 2 / output_spacing[2]);

			output_type::RegionType output_region;
			output_region.SetSize(output_size);
			output_region.SetIndex(output_start);

			gte::Vector<3, double> origin_gte = volumetric_bounding_box.center
				- volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0]
				- volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1]
				- volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];

			output_type::PointType output_origin;
			output_origin[0] = origin_gte[0];
			output_origin[1] = origin_gte[1];
			output_origin[2] = origin_gte[2];

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

			out_volume->SetRegions(output_region);
			out_volume->SetOrigin(output_origin);
			out_volume->SetSpacing(output_spacing);
			out_volume->SetDirection(output_directorion);
			out_volume->Allocate(true);

			acummulation_buffer->SetRegions(output_region);
			acummulation_buffer->SetOrigin(output_origin);
			acummulation_buffer->SetSpacing(output_spacing);
			acummulation_buffer->SetDirection(output_directorion);
			acummulation_buffer->Allocate(true);

			volumes_initiated = true;
			return true;
		} else {

			gte::Vector<3, double> origin_gte = volumetric_bounding_box.center
				- volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0]
				- volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1]
				- volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];

			resampler_output::Pointer resampler = resampler_output::New();

			itk::Matrix<double> transform;
			transform[0][0] = volumetric_bounding_box.axis[0][0];
			transform[1][0] = volumetric_bounding_box.axis[1][0];
			transform[2][0] = volumetric_bounding_box.axis[2][0];

			transform[0][1] = volumetric_bounding_box.axis[0][1];
			transform[1][1] = volumetric_bounding_box.axis[1][1];
			transform[2][1] = volumetric_bounding_box.axis[2][1];

			transform[0][2] = volumetric_bounding_box.axis[0][2];
			transform[1][2] = volumetric_bounding_box.axis[1][2];
			transform[2][2] = volumetric_bounding_box.axis[2][2];

			itk::AffineTransform<double>::Pointer affine_transform = itk::AffineTransform<double>::New();
			resampler->SetTransform(affine_transform);

			itk::NearestNeighborInterpolateImageFunction<output_type>::Pointer interpolator = itk::NearestNeighborInterpolateImageFunction<output_type>::New();
			resampler->SetInterpolator(interpolator);

			output_type::SizeType output_size;
			output_size[0] = std::ceil(volumetric_bounding_box.extent[0] * 2 / output_spacing[0]);
			output_size[1] = std::ceil(volumetric_bounding_box.extent[1] * 2 / output_spacing[1]);
			output_size[2] = std::ceil(volumetric_bounding_box.extent[2] * 2 / output_spacing[2]);

			resampler->SetSize(output_size);
			resampler->SetDefaultPixelValue(0);
			resampler->SetOutputSpacing(output_spacing);
			resampler->SetOutputOrigin(origin_gte[0]);
			resampler->SetOutputDirection(transform);
			resampler->SetOutputStartIndex({ 0,0,0 });
			resampler->SetInput(out_volume);
			output_type::Pointer new_output;

			try {
				resampler->Update();
				new_output = resampler->GetOutput();
			}
			catch (...) {
				return false;
			}

			out_volume = new_output;

			itk::NearestNeighborInterpolateImageFunction<accumulator_type>::Pointer accumulator_interpolator = itk::NearestNeighborInterpolateImageFunction<accumulator_type>::New();
			resampler_accumulator::Pointer resampler_accumulator = resampler_accumulator::New();
			resampler_accumulator->SetTransform(affine_transform);
			resampler_accumulator->SetInterpolator(accumulator_interpolator);
			resampler_accumulator->SetSize(output_size);
			resampler_accumulator->SetDefaultPixelValue(0);
			resampler_accumulator->SetOutputSpacing(output_spacing);
			resampler_accumulator->SetOutputOrigin(origin_gte[0]);
			resampler_accumulator->SetOutputDirection(transform);
			resampler_accumulator->SetOutputStartIndex({ 0,0,0 });
			resampler_accumulator->SetInput(acummulation_buffer);
			accumulator_type::Pointer new_acummulation_buffer;

			try {
				resampler_accumulator->Update();
				new_acummulation_buffer = resampler_accumulator->GetOutput();
			} catch (...) {
				return false;
			}

			acummulation_buffer = new_acummulation_buffer;

			return true;
		}
		return true;
	}
	return true;
};

}
}