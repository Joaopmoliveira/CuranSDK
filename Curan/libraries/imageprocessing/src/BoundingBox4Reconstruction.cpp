#include "imageprocessing/BoundingBox4Reconstruction.h"
#include "imageprocessing/VolumeAlgorithms.h"
#include "utils/Logger.h"
#include <list>
#include <set>

namespace curan {
namespace image {

BoundingBox4Reconstruction::BoundingBox4Reconstruction()
{
	volumes_initiated = false;
}

BoundingBox4Reconstruction::~BoundingBox4Reconstruction()
{
}

struct already_found
{
  std::set<gte::Vector3<double>> & theSet;

  bool operator()(const gte::Vector3<double>& s) const
  {
     return !theSet.insert(s).second;
  }
};

void BoundingBox4Reconstruction::update()
{
	std::vector<gte::Vector3<double>> vertices;

	// We multiply by four because each 
	// image has four courners
	vertices.resize(frame_data.size() * 4);

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
		//std::printf("\tvertex 2 : ( %f %f %f )\n",vertices[increment+1][0],vertices[increment+1][1],vertices[increment+1][2]);

		output_type::IndexType origin_along_width_and_height = { width - 1,height - 1,0 };
		output_type::PointType origin_along_width_and_height_position;
		img->TransformIndexToPhysicalPoint(origin_along_width_and_height, origin_along_width_and_height_position);

		vertices[increment + 2] = gte::Vector3<double>({ origin_along_width_and_height_position[0], origin_along_width_and_height_position[1], origin_along_width_and_height_position[2] });
		//std::printf("\tvertex 3 : ( %f %f %f )\n",vertices[increment+2][0],vertices[increment+2][1],vertices[increment+2][2]);

		output_type::IndexType origin_along_height = { 0,height - 1,0 };
		output_type::PointType origin_along_height_position;
		img->TransformIndexToPhysicalPoint(origin_along_height, origin_along_height_position);

		vertices[increment + 3] = gte::Vector3<double>({ origin_along_height_position[0], origin_along_height_position[1], origin_along_height_position[2] });
		//std::printf("\tvertex 4 : ( %f %f %f )\n",vertices[increment+3][0],vertices[increment+3][1],vertices[increment+3][2]);
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
		std::list<gte::Vector3<double>> list;
		std::copy( vertices.begin(), vertices.end(), std::back_inserter( list ) );
		std::set<gte::Vector3<double>> theSet;
		list.remove_if(already_found{theSet} );
		vertices = std::vector<gte::Vector3<double>>(list.begin(),list.end());
		//for(const auto& vert : vertices)
		//	std::printf("( %f %f %f )\n",vert[0],vert[1],vert[2]);

		bounding_box(vertices.size(), vertices.data(), 4, volumetric_bounding_box, volume);

		//std::cout << "Volumes initiated" << std::endl;
		volumes_initiated = true;
	} else {
		//std::cout << "New image" << std::endl;
		std::array<gte::Vector3<double>, 8> current_corners;
		current_corners[0] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
		current_corners[1] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
		current_corners[2] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
		current_corners[3] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
		current_corners[4] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
		current_corners[5] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
		current_corners[6] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
		current_corners[7] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];

		vertices.insert(vertices.end(), std::begin(current_corners), std::end(current_corners));
		std::list<gte::Vector3<double>> list;
		std::copy( vertices.begin(), vertices.end(), std::back_inserter( list ) );
		std::set<gte::Vector3<double>> theSet;
		list.remove_if(already_found{theSet} );
		vertices = std::vector<gte::Vector3<double>>(list.begin(),list.end());
		/* for(const auto& vert : vertices)
			std::printf("*( %f %f %f )\n",vert[0],vert[1],vert[2]); */
		gte::MinimumVolumeBox3<double, true> bounding_box(0);

		double volume = 0.0;
		bounding_box(vertices.size(), vertices.data(), 4, volumetric_bounding_box, volume);
		//std::printf("Volumetric bounding box center (%f, %f,%f)\n", volumetric_bounding_box.center[0], volumetric_bounding_box.center[1], volumetric_bounding_box.center[2]);
	}

	frame_data.clear();
};

/*

void BoundingBox4Reconstruction::get_final_volume_vertices(array_type& box_data){
	std::array<gte::Vector3<double>, 8> current_corners;
	current_corners[0] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
	current_corners[1] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
	current_corners[2] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
	current_corners[3] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] + volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
	current_corners[4] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
	current_corners[5] = volumetric_bounding_box.center + volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
	current_corners[6] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] + volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];
	current_corners[7] = volumetric_bounding_box.center - volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0] - volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1] - volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];

	box_data = current_corners;
};
*/

void BoundingBox4Reconstruction::add_frames(std::vector<output_type::Pointer>& images_vector)
{
	frame_data.insert(std::end(frame_data), std::begin(images_vector), std::end(images_vector));
};

void BoundingBox4Reconstruction::add_frame(output_type::Pointer image_pointer)
{
	frame_data.push_back(image_pointer);
};


}
}
