#include "imageprocessing/VolumeReconstructorBoxDefiner.h"
#include "imageprocessing/VolumeAlgorithms.h"
#include "utils/Logger.h"

namespace curan {
namespace image {

VolumeReconstructorBoxDefiner::VolumeReconstructorBoxDefiner()
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

VolumeReconstructorBoxDefiner::~VolumeReconstructorBoxDefiner()
{
}

void VolumeReconstructorBoxDefiner::update()
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
	auto& img = frame_data;
	for (auto& img : frame_data)
	{
		auto size = img->GetLargestPossibleRegion().GetSize();
		int height = size[1];
		int width = size[0];

		output_type::PointType origin_position;
		output_type::IndexType origin_pixel = { 0,0,0 };
		img->TransformIndexToPhysicalPoint(origin_pixel, origin_position);

		vertices[increment] = gte::Vector3<double>({ origin_position[0], origin_position[1], origin_position[2] });
		std::printf("image %d - \n\tvertex 1 : ( %f %f %f )\n",counter,vertices[increment][0],vertices[increment][1],vertices[increment][2]);

		output_type::IndexType origin_along_width = { width - 1,0,0 };
		output_type::PointType origin_along_width_position;
		img->TransformIndexToPhysicalPoint(origin_along_width, origin_along_width_position);

		vertices[increment + 1] = gte::Vector3<double>({ origin_along_width_position[0], origin_along_width_position[1], origin_along_width_position[2] });
		std::printf("\tvertex 2 : ( %f %f %f )\n",vertices[increment+1][0],vertices[increment+1][1],vertices[increment+1][2]);

		output_type::IndexType origin_along_width_and_height = { width - 1,height - 1,0 };
		output_type::PointType origin_along_width_and_height_position;
		img->TransformIndexToPhysicalPoint(origin_along_width_and_height, origin_along_width_and_height_position);

		vertices[increment + 2] = gte::Vector3<double>({ origin_along_width_and_height_position[0], origin_along_width_and_height_position[1], origin_along_width_and_height_position[2] });
		std::printf("\tvertex 3 : ( %f %f %f )\n",vertices[increment+2][0],vertices[increment+2][1],vertices[increment+2][2]);

		output_type::IndexType origin_along_height = { 0,height - 1,0 };
		output_type::PointType origin_along_height_position;
		img->TransformIndexToPhysicalPoint(origin_along_height, origin_along_height_position);

		vertices[increment + 3] = gte::Vector3<double>({ origin_along_height_position[0], origin_along_height_position[1], origin_along_height_position[2] });
		std::printf("\tvertex 4 : ( %f %f %f )\n",vertices[increment+3][0],vertices[increment+3][1],vertices[increment+3][2]);
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

		std::cout << "Volumes initiated" << std::endl;
		volumes_initiated = true;
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
		//std::cout << vertices.size() << std::endl;
	}

	frame_data.clear();
};

void VolumeReconstructorBoxDefiner::set_clipping_bounds(std::array<double, 2> inclipRectangleOrigin, std::array<double, 2> inclipRectangleSize)
{
	clipRectangleOrigin = inclipRectangleOrigin;
	clipRectangleSize = inclipRectangleSize;
}

void VolumeReconstructorBoxDefiner::get_final_volume_vertices(std::array<gte::Vector3<double>, 8> box_data){
	std::array<gte::Vector3<double>, 8> current_corners;
	current_corners[0] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
	current_corners[1] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
	current_corners[2] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
	current_corners[3] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
	current_corners[4] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
	current_corners[5] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
	current_corners[6] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
	current_corners[7] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];

	std::cout << current_corners[0][0] << std::endl;
	std::cout << current_corners[0][1] << std::endl;
	std::cout << current_corners[0][2] << std::endl;
	std::cout << current_corners[1][0] << std::endl;
	std::cout << current_corners[1][1] << std::endl;
	std::cout << current_corners[1][2] << std::endl;
	std::cout << current_corners[2][0] << std::endl;
	std::cout << current_corners[2][1] << std::endl;
	std::cout << current_corners[2][2] << std::endl;
	std::cout << current_corners[3][0] << std::endl;
	std::cout << current_corners[3][1] << std::endl;
	std::cout << current_corners[3][2] << std::endl;
	std::cout << current_corners[4][0] << std::endl;
	std::cout << current_corners[4][1] << std::endl;
	std::cout << current_corners[4][2] << std::endl;
	std::cout << current_corners[5][0] << std::endl;
	std::cout << current_corners[5][1] << std::endl;
	std::cout << current_corners[5][2] << std::endl;
	std::cout << current_corners[6][0] << std::endl;
	std::cout << current_corners[6][1] << std::endl;
	std::cout << current_corners[6][2] << std::endl;
	std::cout << current_corners[7][0] << std::endl;
	std::cout << current_corners[7][1] << std::endl;
	std::cout << current_corners[7][2] << std::endl;
	box_data = current_corners;
};

void VolumeReconstructorBoxDefiner::add_frames(std::vector<output_type::Pointer>& images_vector)
{
	frame_data.insert(std::end(frame_data), std::begin(images_vector), std::end(images_vector));
};

void VolumeReconstructorBoxDefiner::add_frame(output_type::Pointer image_pointer)
{
	frame_data.push_back(image_pointer);
};


}
}
