#include "imageprocessing/BoundingBox4Reconstruction.h"
#include "imageprocessing/VolumeAlgorithms.h"
#include "utils/Logger.h"
#include <list>
#include <set>

namespace curan {
namespace image {

BoundingBox4Reconstruction::BoundingBox4Reconstruction() 
{ }

BoundingBox4Reconstruction::~BoundingBox4Reconstruction()
{ }

struct already_found{
  std::set<gte::Vector3<double>> & theSet;

  bool operator()(const gte::Vector3<double>& s) const
  {
     return !theSet.insert(s).second;
  }
};

void BoundingBox4Reconstruction::update()
{
	std::vector<gte::Vector3<double>> vertices;
	vertices.resize(frame_data.size() * 4);

	int increment = 0;
	int counter = 0 ;

	for (auto& img : frame_data){
		auto size = img->GetLargestPossibleRegion().GetSize();
		int height = size[1];
		int width = size[0];

		output_type::PointType origin_position;
		output_type::IndexType origin_pixel = { 0,0,0 };
		img->TransformIndexToPhysicalPoint(origin_pixel, origin_position);

		vertices[increment] = gte::Vector3<double>({ origin_position[0], origin_position[1], origin_position[2] });

		output_type::IndexType origin_along_width = { width - 1,0,0 };
		output_type::PointType origin_along_width_position;
		img->TransformIndexToPhysicalPoint(origin_along_width, origin_along_width_position);

		vertices[increment + 1] = gte::Vector3<double>({ origin_along_width_position[0], origin_along_width_position[1], origin_along_width_position[2] });

		output_type::IndexType origin_along_width_and_height = { width - 1,height - 1,0 };
		output_type::PointType origin_along_width_and_height_position;
		img->TransformIndexToPhysicalPoint(origin_along_width_and_height, origin_along_width_and_height_position);

		vertices[increment + 2] = gte::Vector3<double>({ origin_along_width_and_height_position[0], origin_along_width_and_height_position[1], origin_along_width_and_height_position[2] });

		output_type::IndexType origin_along_height = { 0,height - 1,0 };
		output_type::PointType origin_along_height_position;
		img->TransformIndexToPhysicalPoint(origin_along_height, origin_along_height_position);

		vertices[increment + 3] = gte::Vector3<double>({ origin_along_height_position[0], origin_along_height_position[1], origin_along_height_position[2] });
		increment += 4;
		++counter;
	};

	std::vector<gte::Vector3<double>> previous_and_current_verticies;
	previous_and_current_verticies.insert( previous_and_current_verticies.end(), vertices.begin(), vertices.end() );
	previous_and_current_verticies.insert( previous_and_current_verticies.end(), current_vertices.begin(), current_vertices.end());
	gte::ConvexHull3<double> convex_hull;
	convex_hull(previous_and_current_verticies,0);
	current_vertices.clear();
	auto vert = convex_hull.GetVertices();
	for(const auto& ind : vert)
		current_vertices.push_back(previous_and_current_verticies[ind]);

	double volume = 0.0;
	gte::MinimumVolumeBox3<double, true> bounding_box{0};
	bounding_box(current_vertices.size(), current_vertices.data(), 4, volumetric_bounding_box, volume);
	frame_data.clear();
};

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
