#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/DicomDisplay.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Drawable.h"

#include <unordered_map>
#include <optional>
#include <functional>


#include "utils/Lockable.h"
#include "userinterface/widgets/SignalProcessor.h"
#include "userinterface/widgets/ImageWrapper.h"
#include "userinterface/widgets/ComputeImageBounds.h"
#include "utils/Overloading.h"

#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkOrientImageFilter.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include <Mathematics/ConvexPolyhedron3.h>
#include <Mathematics/DistPoint3ConvexPolyhedron3.h>

using DicomPixelType = unsigned short;
using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using DICOMImageType = itk::Image<DicomPixelType, Dimension>;


bool IsPointInConvexMesh(const gte::Vector3<double>& point, 
                         const gte::ConvexPolyhedron3<double>& mesh,
                         double epsilon = 0)
{
    for (const auto& plane : mesh.planes)
    {
        gte::Vector3<double> normal{plane[0],plane[1],plane[2]};
        gte::Normalize(normal,true);
        double signedDistance = gte::Dot<3,double>(normal, point) + plane[3];
        if (signedDistance > epsilon)
        {
            return false;
        }
    }
    std::printf("success!\n");
    return true;
}

ImageType::Pointer create_volume(auto& geometries) {
//Sphere
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



    ImageType::SpacingType spacing;
    spacing[0] = 1.0;
 	spacing[1] = 1.0;
 	spacing[2] = 1.0;

    std::array<double,2> xbounds{1000000.0,-10.0};
    std::array<double,2> ybounds{1000000.0,-10.0};
    std::array<double,2> zbounds{1000000.0,-10.0};

    for(auto& geom : geometries)
        for(auto ver : geom.vertices){
            if(ver[0] < xbounds[0])
                xbounds[0] = ver[0];
            if(ver[1] < ybounds[0])
                ybounds[0] = ver[1];
            if(ver[2] < zbounds[0])
                zbounds[0] = ver[2];
            if(ver[0] > xbounds[1])
                xbounds[1] = ver[0];
            if(ver[1] > ybounds[1])
                ybounds[1] = ver[1];
            if(ver[2] > zbounds[1])
                zbounds[1] = ver[2];
        }

 	image_origin[0] = xbounds[0];
 	image_origin[1] = ybounds[0];
 	image_origin[2] = zbounds[0];

    if(xbounds[1]-xbounds[0]<0.0 || xbounds[1]-xbounds[0] > 500)
        throw std::runtime_error("cannot parse such large volume");

    if(ybounds[1]-ybounds[0]<0.0 || ybounds[1]-ybounds[0] > 500)
        throw std::runtime_error("cannot parse such large volume");

    if(zbounds[1]-zbounds[0]<0.0 || zbounds[1]-zbounds[0] > 500)
        throw std::runtime_error("cannot parse such large volume");

    std::printf("sizes x[%.2f %.2f] y[%.2f %.2f] z[%.2f %.2f]\n",xbounds[0],xbounds[1],ybounds[0],ybounds[1],zbounds[0],zbounds[1]);

    ImageType::SizeType size = { { (int)(xbounds[1]-xbounds[0]), (int)(ybounds[1]-ybounds[0]), (int)(zbounds[1]-zbounds[0]) } };
    ImageType::IndexType index = { { 0, 0, 0 } };
    ImageType::RegionType region;
    region.SetSize(size);
    region.SetIndex(index);
    ImageType::Pointer image = ImageType::New();
    image->SetRegions(region);
    image->SetDirection(image_orientation);
    image->SetSpacing(spacing);
    image->SetOrigin(image_origin);
    image->Allocate(true); // initialize buffer to zero

    using Iterator = itk::ImageRegionIterator<ImageType>;

    ImageType::IndexType idx = {{0,0,0}};

    image->TransformIndexToPhysicalPoint(idx,image_origin);

    Iterator it(image, region);
    
    itk::Point<double,3> current_pixel;
    gte::Vector3<double> point;
    itk::Point<double,3> max_pixel_position;

    for (it.GoToBegin(); !it.IsAtEnd(); ++it)
    {
        ImageType::IndexType current_idx = it.GetIndex();

        image->TransformIndexToPhysicalPoint(current_idx,current_pixel);
        point[0] = current_pixel[0];
        point[1] = current_pixel[1];
        point[2] = current_pixel[2];
        for(auto& geom : geometries){
            if(IsPointInConvexMesh(point,geom)){
                it.Set(255.0);
                break;
            }
        }
    }
    return image;
}

int main()
{
	try
	{
        using Rational = gte::BSRational<gte::UIntegerAP32>;
		using namespace curan::ui;
		IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();
		
        Eigen::Matrix<double,4,4> transf = Eigen::Matrix<double,4,4>::Identity();
        transf(0,3) = 10;
        transf(1,3) = 10;
        transf(2,3) = 10;
		curan::geometry::ClosedCylinder cilinder{2,10,10.0,15.0};
        cilinder.transform(transf);
        curan::geometry::Cube cube{10.0,10.0,10.0};
        transf(0,3) = 5;
        transf(1,3) = 10;
        transf(2,3) = 1;
        cube.transform(transf);
        //the piramid also needs to be scaled beyond moved 
        curan::geometry::Piramid piramid{}; 
        Eigen::Matrix<double,4,4> scale = Eigen::Matrix<double,4,4>::Identity();
        scale(0,0) = 4;
        scale(1,1) = 4;
        scale(2,2) = 4;
        piramid.transform(scale);
        transf(0,3) = 20;
        transf(1,3) = 10;
        transf(2,3) = 5;
        piramid.transform(transf);

		DisplayParams param{std::move(context), 2200, 1200};
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
        DicomVolumetricMask<std::uint8_t> mask{nullptr};
		auto geometry_name = mask.add_geometry(cube,SK_ColorRED);
        geometry_name = mask.add_geometry(cilinder,SK_ColorCYAN); 
        geometry_name = mask.add_geometry(piramid,SK_ColorYELLOW);

        std::vector<gte::ConvexPolyhedron3<double>> geometries;

        for(auto& [description,geom] : mask.geometries()){
            auto& [geo,color] = geom;
            std::vector<gte::Vector3<double>> vertex;
            std::vector<int32_t> indicies;
            for(auto ver : geo.geometry.vertices)
                vertex.push_back(gte::Vector3<double>{(double)ver[0],(double)ver[1],(double)ver[2]});
            for(auto triangle : geo.geometry.triangles){
                indicies.push_back(triangle[0]); 
                indicies.push_back(triangle[1]); 
                indicies.push_back(triangle[2]); 
            }
            gte::ConvexPolyhedron3<double> geom(std::move(vertex),std::move(indicies),true,true);
            geometries.emplace_back(geom);
        }
        auto volume = create_volume(geometries);

        //now I need to generate a ITK image that constains these shapes, transverse through the pixels and change their color
        //depending if the point is contained inside the shape or not
        mask.update_volume(volume,DicomVolumetricMask<std::uint8_t>::Policy::UPDATE_GEOMETRIES | DicomVolumetricMask<std::uint8_t>::Policy::UPDATE_POINTS);

		std::unique_ptr<curan::ui::DicomViewer<std::uint8_t>> image_display = curan::ui::DicomViewer<std::uint8_t>::make(resources, &mask, curan::ui::Direction::Z);

		auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
		*container << std::move(image_display);

		curan::ui::Page page{std::move(container), SK_ColorBLACK};

		ConfigDraw config{&page};

		while (!glfwWindowShouldClose(viewer->window))
		{
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface *pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas *canvas = pointer_to_surface->getCanvas();
			if (viewer->was_updated())
			{
				page.update_page(viewer.get());
				viewer->update_processed();
			}
			page.draw(canvas);
			auto signals = viewer->process_pending_signals();
			if (!signals.empty())
				page.propagate_signal(signals.back(), &config);
			glfwPollEvents();

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch (const std::exception &e)
	{
		std::cout << "Exception thrown:" << e.what() << "\n";
	}
	catch (...)
	{
		std::cout << "Failed to create window for unknown reason\n";
		return 1;
	}
}