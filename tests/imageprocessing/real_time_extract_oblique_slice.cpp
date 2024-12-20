#include "itkImage.h"
#include "itkImageFileReader.h"

#include "itkResampleImageFilter.h"

#include "itkEuler3DTransform.h"
#include "itkNearestNeighborInterpolateImageFunction.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include "itkImage.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkScaleTransform.h"
#include "itkAffineTransform.h"
#include "itkEuler3DTransform.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/DynamicTexture.h"
#include <optional>
#include <chrono>
#include <thread>
#include "Mathematics/IntrPlane3OrientedBox3.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "rendering/Sphere.h"


constexpr unsigned int Dimension_in = 3;
constexpr unsigned int Dimension_out = 3;
using InputPixelType = unsigned char;
using OutputPixelType = unsigned char;
using DicomPixelType = unsigned short;

using InterPixelType = float;

using InputImageType = itk::Image<InterPixelType, Dimension_in>;
using OutputImageType = itk::Image<OutputPixelType, Dimension_out>;
using DICOMImageType = itk::Image<DicomPixelType, Dimension_in>;

using InterImageType = itk::Image<InterPixelType, Dimension_out>;

// using TransformType = itk::AffineTransform<double, Dimension_in>;
using TransformType = itk::Euler3DTransform<double>;

using ReaderType = itk::ImageFileReader<InputImageType>;


void obtain_rot_matrix_by_angles(Eigen::Matrix<double,3,3>& R_matrix, Eigen::Matrix<double,3,1>& rotaton_angles){

    typedef double T;

    auto a = rotaton_angles[0];
    auto b = rotaton_angles[1];
    auto c = rotaton_angles[2];

    R_matrix << cos(b)*cos(c), sin(a)*sin(b)*cos(c)-cos(a)*sin(c), cos(a)*sin(b)*cos(c)+sin(a)*sin(c),
            cos(b)*sin(c), sin(a)*sin(b)*sin(c)+cos(a)*cos(c), cos(a)*sin(b)*sin(c)-sin(a)*cos(c),
            -sin(b), sin(a)*cos(b), cos(a)*cos(b);
}

void exclude_row_matrix(Eigen::MatrixXd& matrix, size_t row_to_remove){
    int numRows = static_cast<int>(matrix.rows()) - 1;
    int numCols = static_cast<int>(matrix.cols());

    if(row_to_remove < numRows){
        matrix.block(row_to_remove,0,numRows-row_to_remove,numCols) = matrix.bottomRows(numRows-row_to_remove);
    }

    matrix.conservativeResize(numRows,numCols);
}


void calculate_image_centroid(const Eigen::Matrix<double,3,1>& volume_size_mm , Eigen::Matrix<double, 3, 1> &centroid, const Eigen::Matrix<double, 3, 3> &R_ImageToVolume, const Eigen::Matrix<double, 3, 1> &needle_tip_transformed_to_volume_space)
{
    Eigen::Matrix<double, 8, 3> vol_vertices_coords;

    vol_vertices_coords(0, 0) = 0.0;
    vol_vertices_coords(0, 1) = 0.0;
    vol_vertices_coords(0, 2) = 0.0;

    vol_vertices_coords(1, 0) = volume_size_mm[0];
    vol_vertices_coords(1, 1) = 0.0;
    vol_vertices_coords(1, 2) = 0.0;

    vol_vertices_coords(2, 0) = volume_size_mm[0];
    vol_vertices_coords(2, 1) = volume_size_mm[1];
    vol_vertices_coords(2, 2) = 0.0;

    vol_vertices_coords(3, 0) = 0.0;
    vol_vertices_coords(3, 1) = volume_size_mm[1];
    vol_vertices_coords(3, 2) = 0.0;

    vol_vertices_coords(4, 0) = 0.0;
    vol_vertices_coords(4, 1) = 0.0;
    vol_vertices_coords(4, 2) = volume_size_mm[2];

    vol_vertices_coords(5, 0) = volume_size_mm[0];
    vol_vertices_coords(5, 1) = 0.0;
    vol_vertices_coords(5, 2) = volume_size_mm[2];

    vol_vertices_coords(6, 0) = volume_size_mm[0];
    vol_vertices_coords(6, 1) = volume_size_mm[1];
    vol_vertices_coords(6, 2) = volume_size_mm[2];

    vol_vertices_coords(7, 0) = 0.0;
    vol_vertices_coords(7, 1) = volume_size_mm[1];
    vol_vertices_coords(7, 2) = volume_size_mm[2];

    double plane_equation[4];
    plane_equation[0] = R_ImageToVolume(0, 2);
    plane_equation[1] = R_ImageToVolume(1, 2);
    plane_equation[2] = R_ImageToVolume(2, 2);
    plane_equation[3] = -(R_ImageToVolume.col(2).transpose() * needle_tip_transformed_to_volume_space)(0, 0);

    constexpr double allowed_error = 1e-10;

    Eigen::MatrixXd intersections;
    intersections.resize(12, 3);

    size_t i = 0;
    for (size_t j = 0; j < vol_vertices_coords.rows(); ++j)
    {
        if (std::abs(vol_vertices_coords(j, 0)) < allowed_error){
            double majored_value = ((std::abs(plane_equation[0])>= allowed_error) ? plane_equation[0] : allowed_error);
            intersections(i, 0) = -(plane_equation[1] * vol_vertices_coords(j, 1) + plane_equation[2] * vol_vertices_coords(j, 2) + plane_equation[3]) / majored_value;
            intersections(i, 1) = vol_vertices_coords(j, 1);
            intersections(i, 2) = vol_vertices_coords(j, 2);
            i = i + 1;
        }
        if (std::abs(vol_vertices_coords(j, 1)) < allowed_error ){
            double majored_value = ((std::abs(plane_equation[1])>= allowed_error) ? plane_equation[1] : allowed_error);
            intersections(i, 1) = -(plane_equation[0] * vol_vertices_coords(j, 0) + plane_equation[2] * vol_vertices_coords(j, 2) + plane_equation[3]) / majored_value;
            intersections(i, 0) = vol_vertices_coords(j, 0);
            intersections(i, 2) = vol_vertices_coords(j, 2);
            i = i + 1;
        }

        if (std::abs(vol_vertices_coords(j, 2)) < allowed_error){
            double majored_value = ((std::abs(plane_equation[2])>= allowed_error) ? plane_equation[2] : allowed_error);
            intersections(i, 2) = -(plane_equation[1] * vol_vertices_coords(j, 1) + plane_equation[0] * vol_vertices_coords(j, 0) + plane_equation[3]) / majored_value;
            intersections(i, 0) = vol_vertices_coords(j, 0);
            intersections(i, 1) = vol_vertices_coords(j, 1);
            i = i + 1;
        }
    }

    size_t initial_number_of_points = static_cast<int>(intersections.rows())-1;
    bool no_rows = false;
    for (int i = initial_number_of_points; i >= 0 ; --i)
    {
        if (intersections(i, 0) > volume_size_mm[0] + allowed_error || intersections(i, 0) < 0.0 - allowed_error 
                 || intersections(i, 1) > volume_size_mm[1] + allowed_error || intersections(i, 1) < 0.0 - allowed_error 
                 || intersections(i, 2) > volume_size_mm[2] + allowed_error || intersections(i, 2) < 0.0 - allowed_error ){
            if(intersections.rows()==1){
                no_rows = true;
                break;
            }  
            exclude_row_matrix(intersections,i);
        }
    }
    
    centroid = (no_rows) ?  needle_tip_transformed_to_volume_space  : intersections.colwise().mean();
}

InputImageType::Pointer load_dicom() {
    
    using ReaderTypeDicom = itk::ImageSeriesReader<DICOMImageType>;
    auto reader = ReaderTypeDicom::New();

    using ImageIOType = itk::GDCMImageIO;
    auto dicomIO = ImageIOType::New();

    reader->SetImageIO(dicomIO);

    using NamesGeneratorType = itk::GDCMSeriesFileNames;
    auto nameGenerator = NamesGeneratorType::New();

    nameGenerator->SetUseSeriesDetails(true);
    nameGenerator->AddSeriesRestriction("0008|0021");

    std::string dirName_input{CURAN_COPIED_RESOURCE_PATH "/dicom_sample/ST983524"};
    nameGenerator->SetDirectory(dirName_input);

    using SeriesIdContainer = std::vector<std::string>;

    const SeriesIdContainer &seriesUID = nameGenerator->GetSeriesUIDs();

    auto seriesItr = seriesUID.begin();
    auto seriesEnd = seriesUID.end();
    while (seriesItr != seriesEnd)
    {
        std::cout << seriesItr->c_str() << std::endl;
        ++seriesItr;
    }

    std::string seriesIdentifier;
    seriesIdentifier = seriesUID.begin()->c_str();

    using FileNamesContainer = std::vector<std::string>;
    FileNamesContainer fileNames;

    fileNames = nameGenerator->GetFileNames(seriesIdentifier);

    reader->SetFileNames(fileNames);

    using RescaleType = itk::RescaleIntensityImageFilter<DICOMImageType, DICOMImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(reader->GetOutput());
    rescale->SetOutputMinimum(0);
    rescale->SetOutputMaximum(itk::NumericTraits<InputPixelType>::max());

    using FilterType = itk::CastImageFilter<DICOMImageType, InputImageType>;
    auto filter = FilterType::New();
    filter->SetInput(rescale->GetOutput());

    try
    {
        filter->Update();
    }
    catch (const itk::ExceptionObject &ex)
    {
        std::cout << ex << std::endl;
        return nullptr;
    }

    return filter->GetOutput();
};


void create_volume(InputImageType::Pointer image) {
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

 	image_origin[0] = 0.0;
 	image_origin[1] = 0.0;
 	image_origin[2] = 0.0;

    InputImageType::SpacingType spacing;
    spacing[0] = 1.0;
 	spacing[1] = 1.0;
 	spacing[2] = 1.0;


    InputImageType::SizeType size = { { 100, 100, 100 } };
    InputImageType::IndexType index = { { 0, 0, 0 } };
    InputImageType::IndexType max_index = { { 99, 99, 99 } };
    InputImageType::RegionType region;
    region.SetSize(size);
    region.SetIndex(index);
    image->SetRegions(region);
    image->SetDirection(image_orientation);
    image->SetSpacing(spacing);
    image->SetOrigin(image_origin);
    image->Allocate(true); // initialize buffer to zero

    using Iterator = itk::ImageRegionIterator<InputImageType>;

    InputImageType::IndexType idx = {{0,0,0}};

    image->TransformIndexToPhysicalPoint(idx,image_origin);

    Iterator it(image, region);

    itk::Point<double,3> sphere_center;
    sphere_center[0] = 0.0;
    sphere_center[1] = 0.0;
    sphere_center[2] = 0.0;
    
    itk::Point<double,3> current_pixel;

    itk::Point<double,3> max_pixel_position;
    image->TransformIndexToPhysicalPoint(max_index, max_pixel_position);

    auto max_radius = max_pixel_position[0]*max_pixel_position[0] + max_pixel_position[1]*max_pixel_position[1] + max_pixel_position[2]*max_pixel_position[2];

    for (it.GoToBegin(); !it.IsAtEnd(); ++it)
    {
        InputImageType::IndexType current_idx = it.GetIndex();
        image->TransformIndexToPhysicalPoint(current_idx,current_pixel);
        double squared_sphere_radious = ((current_pixel[0]-sphere_center[0])*(current_pixel[0]-sphere_center[0])+(current_pixel[1]-sphere_center[1])*(current_pixel[1]-sphere_center[1])+(current_pixel[2]-sphere_center[2])*(current_pixel[2]-sphere_center[2]));
        it.Set((squared_sphere_radious/max_radius)*255.0);
        
    }
    };


void updateBaseTexture2D(vsg::vec4Array2D &image, OutputImageType::Pointer image_to_render)
{
    try
    {
        int x, y, z;
        auto input = image_to_render;
        auto size = input->GetLargestPossibleRegion().GetSize();
        x = size[0];
        y = size[1];
        z = size[2];

        unsigned char *scaller_buffer = (unsigned char *)image_to_render->GetBufferPointer();

        for (size_t r = 0; r < image.height(); ++r)
        {
            using value_type = typename vsg::vec4Array2D::value_type;
            value_type *ptr = &image.at(0, r);
            for (size_t c = 0; c < image.width(); ++c)
            {
                auto val = *scaller_buffer / 255.0;
                ptr->r = val;
                ptr->g = val;
                ptr->b = val;
                ptr->a = 1.0f;
                ++ptr;
                ++scaller_buffer;
            }
        }
    }
    catch (std::exception &e)
    {
        std::cout << "exception : " << e.what() << std::endl;
        ;
    }
}

void updateBaseTexture3D(vsg::floatArray3D &image, InputImageType::Pointer image_to_render)
{
    using FilterType = itk::CastImageFilter<InputImageType, InputImageType>;
    auto filter = FilterType::New();
    filter->SetInput(image_to_render);

    using RescaleType = itk::RescaleIntensityImageFilter<InputImageType, InputImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(filter->GetOutput());
    rescale->SetOutputMinimum(0.0);
    rescale->SetOutputMaximum(1.0);

    try
    {
        rescale->Update();
    }
    catch (const itk::ExceptionObject &e)
    {
        std::cerr << "Error: " << e << std::endl;
        throw std::runtime_error("error");
    }

    InputImageType::Pointer out = rescale->GetOutput();

    using IteratorType = itk::ImageRegionIteratorWithIndex<InputImageType>;
    IteratorType outputIt(out, out->GetRequestedRegion());
    for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt)
    {
        InputImageType::IndexType idx = outputIt.GetIndex();
        image.set(idx[0], idx[1], idx[2], outputIt.Get());
    }
}

void resampler(curan::renderable::Sphere* sphere,curan::renderable::DynamicTexture *texture, OutputImageType::Pointer& output, InputImageType::Pointer volume, Eigen::Matrix<double,3,1>& needle_tip, Eigen::Matrix<double,3,3>& R_ImageToWorld, double& image_size, double& image_spacing)
{
    using FilterType = itk::ResampleImageFilter<InputImageType, OutputImageType>;
    auto filter = FilterType::New();

    using InterpolatorType = itk::LinearInterpolateImageFunction<InputImageType, double>;
    //using InterpolatorType = itk::NearestNeighborInterpolateImageFunction<InputImageType, double>;
    auto interpolator = InterpolatorType::New();
    filter->SetInterpolator(interpolator);
    filter->SetDefaultPixelValue(100);

    auto input = volume;
    filter->SetInput(input);

    itk::Vector<double, 3> spacing;
    spacing[0] = image_spacing;
    spacing[1] = image_spacing;
    spacing[2] = image_spacing;
    filter->SetOutputSpacing(spacing);
    
    itk::Size<3> size;
    size[0] = image_size;
    size[1] = image_size;
    size[2] = 1;
    filter->SetSize(size);

    auto volume_origin = input->GetOrigin();
    Eigen::Matrix<double,3,1> volume_originn;
    volume_originn[0] = volume_origin[0];
    volume_originn[1] = volume_origin[1];
    volume_originn[2] = volume_origin[2];


    auto volume_direction = input->GetDirection();
    Eigen::Matrix<double,3,3> R_volumeToWorld;
    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            R_volumeToWorld(col, row) = volume_direction(col, row);



    // obtain needle tip coordinates in the volume reference frame
    Eigen::Matrix<double,3,1> needle_tip_transformed_to_volume_space = R_volumeToWorld.transpose() * needle_tip - R_volumeToWorld.transpose() * volume_originn;
    //Eigen::Matrix<double,3,1> needle_tip_transformed_to_volume_space = needle_tip;

    
    // obtain rotation matrix between image and volume
    Eigen::Matrix<double,3,3> R_ImageToVolume = R_volumeToWorld.transpose() * R_ImageToWorld;
    Eigen::Matrix<double,3,1> centroid;

    auto volume_size = volume->GetLargestPossibleRegion().GetSize();
    auto volume_spacing = volume->GetSpacing();

    Eigen::Matrix<double,3,1> volume_size_mm;
    volume_size_mm[0] = volume_size.GetSize()[0] * volume_spacing[0];
    volume_size_mm[1] = volume_size.GetSize()[1] * volume_spacing[1];
    volume_size_mm[2] = volume_size.GetSize()[2] * volume_spacing[2];
    calculate_image_centroid(volume_size_mm, centroid, R_ImageToVolume, needle_tip_transformed_to_volume_space); 


    Eigen::Matrix<double,3,1> origin_to_centroid_image_vector;
    origin_to_centroid_image_vector[0] = image_size * image_spacing * 0.5;
    origin_to_centroid_image_vector[1] = image_size * image_spacing * 0.5;
    origin_to_centroid_image_vector[2] = 0.0;

    Eigen::Matrix<double,3,1> centroid_in_word_space = R_volumeToWorld * centroid + volume_originn;
    auto mat = vsg::translate(centroid_in_word_space[0]*1e-3,centroid_in_word_space[1]*1e-3,centroid_in_word_space[2]*1e-3);
    
    sphere->update_transform(mat);

    Eigen::Matrix<double,3,1> image_origin_vol_ref;
    image_origin_vol_ref = centroid - R_ImageToVolume * origin_to_centroid_image_vector;

    Eigen::Matrix<double,3,1> image_origin_world_ref;
    image_origin_world_ref = R_volumeToWorld * image_origin_vol_ref + volume_originn;

    itk::Point<double, 3> origin;
    origin[0] = image_origin_world_ref[0];
    origin[1] = image_origin_world_ref[1];
    origin[2] = image_origin_world_ref[2];
    filter->SetOutputOrigin(origin); 

    itk::Matrix<double> image_direction;
    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            image_direction(col, row) = R_ImageToWorld(col, row);

    filter->SetOutputDirection(image_direction);

    using TransformType = itk::IdentityTransform<double, 3>;
    auto transform = TransformType::New();
    filter->SetTransform(transform);


    try
    {
        filter->Update();
    }
    catch (const itk::ExceptionObject &e)
    {
        std::string result = "Failure to update the filter" + std::string{e.what()};
        std::cout << result;
    }


    OutputImageType::Pointer local_output = filter->GetOutput();

    auto new_origin = local_output->GetOrigin();
    auto image_homogenenous_transformation = vsg::translate(0.0, 0.0, 0.0);
    image_homogenenous_transformation(3, 0) = (new_origin[0]) / 1000.0;
    image_homogenenous_transformation(3, 1) = (new_origin[1]) / 1000.0;
    image_homogenenous_transformation(3, 2) = (new_origin[2]) / 1000.0;

    auto image_final_direction = local_output->GetDirection();
    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            image_homogenenous_transformation(col, row) = image_final_direction(row, col);


    texture->update_transform(image_homogenenous_transformation);
    auto updater_image = [local_output](vsg::vec4Array2D &image){ 
            updateBaseTexture2D(image, local_output);
    };
    texture->update_texture(updater_image);
    output = local_output;

    using Iterator2 = itk::ImageRegionIterator<OutputImageType>;

    auto regionn = local_output->GetRequestedRegion();

    Iterator2 itr(local_output, regionn);

    OutputImageType::IndexType volume_idx;
    itk::Point<double,3> current_point;

    double volume_pixel_value{};
    double slice_pixel_value{};
    

    double max_pixel_value_difference = 0.0;
    double min_pixel_value_difference = 10000.0;
    double accumulator = 0.0;
    double mean = 0.0;
    size_t num_pixels = 0;

    auto input_size = volume->GetLargestPossibleRegion().GetSize();

    for (itr.GoToBegin(); !itr.IsAtEnd(); ++itr){
        OutputImageType::IndexType current_idx = itr.GetIndex();
        output->TransformIndexToPhysicalPoint(current_idx, current_point);
        slice_pixel_value = output->GetPixel(current_idx);
        volume->TransformPhysicalPointToIndex(current_point, volume_idx);
        if(volume_idx[0] >= 0 && volume_idx[0] < input_size[0] 
            && volume_idx[1] >= 0 && volume_idx[1] < input_size[1] 
            && volume_idx[2] >= 0 && volume_idx[2] < input_size[2] ){

                volume_pixel_value = volume->GetPixel(volume_idx);
                double difference = std::abs(slice_pixel_value-volume_pixel_value);
                accumulator += difference;
                max_pixel_value_difference = (difference>max_pixel_value_difference) ? difference : max_pixel_value_difference;
                min_pixel_value_difference = (difference<min_pixel_value_difference) ? difference : min_pixel_value_difference;
                ++num_pixels;
        }
    }
    if(num_pixels>1)
        mean = accumulator/num_pixels;
    std::printf("min (%.4f) mean(%.4f) max(%.4f) npixels(%llu)\n",min_pixel_value_difference,mean,max_pixel_value_difference,num_pixels);

}


int main(int argc, char *argv[])
{
    InputImageType::Pointer input_volume = load_dicom();
    if(input_volume.GetPointer()==nullptr){
        return 1;
    }

    auto volume_size = input_volume->GetLargestPossibleRegion().GetSize();
    auto volume_spacing = input_volume->GetSpacing();

    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "myviewer";
    curan::renderable::Window::WindowSize size_1{1000, 800};
    info.window_size = size_1;
    curan::renderable::Window window{info};

    curan::renderable::Volume::Info volumeinfo;
    volumeinfo.width = volume_size.GetSize()[0];
    volumeinfo.height = volume_size.GetSize()[1];
    volumeinfo.depth = volume_size.GetSize()[2];
    volumeinfo.spacing_x = volume_spacing[0];
    volumeinfo.spacing_y = volume_spacing[1];
    volumeinfo.spacing_z = volume_spacing[2];

    auto volume_to_render = curan::renderable::Volume::make(volumeinfo);
    window << volume_to_render;

    auto direction = input_volume->GetDirection();
    auto origin = input_volume->GetOrigin();
    auto volume_homogenenous_transformation = vsg::translate(0.0, 0.0, 0.0);
    volume_homogenenous_transformation(3, 0) = origin[0] / 1000.0;
    volume_homogenenous_transformation(3, 1) = origin[1] / 1000.0;
    volume_homogenenous_transformation(3, 2) = origin[2] / 1000.0;

    Eigen::Matrix<double,3,1> originn = {origin[0], origin[1], origin[2]};

    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            volume_homogenenous_transformation(col, row) = direction(row, col);

    Eigen::Matrix<double,3,3> volume_directionn;
    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            volume_directionn(col, row) = direction(row, col);

    volume_to_render->cast<curan::renderable::Volume>()->update_transform(volume_homogenenous_transformation);

    auto casted_volume = volume_to_render->cast<curan::renderable::Volume>();
    auto updater = [input_volume](vsg::floatArray3D &image)
    { updateBaseTexture3D(image, input_volume); };
    casted_volume->update_volume(updater);


    double image_spacing = std::min(std::min(volume_spacing[0], volume_spacing[1]), volume_spacing[2]);


    Eigen::Matrix<double,3,1> volume_size_transformed_to_minimun_spacing;
    volume_size_transformed_to_minimun_spacing[0] = (volume_size[0] * volume_spacing[0]) / image_spacing;
    volume_size_transformed_to_minimun_spacing[1] = (volume_size[1] * volume_spacing[1]) / image_spacing;
    volume_size_transformed_to_minimun_spacing[2] = (volume_size[2] * volume_spacing[2]) / image_spacing;

    double image_size = std::sqrt(std::pow(volume_size_transformed_to_minimun_spacing[0],2)+std::pow(volume_size_transformed_to_minimun_spacing[1],2)+std::pow(volume_size_transformed_to_minimun_spacing[2],2));


    curan::renderable::DynamicTexture::Info infotexture;
    infotexture.height = image_size;
    infotexture.width = image_size;
    infotexture.spacing = {image_spacing / 1000, image_spacing / 1000, image_spacing / 1000};
    infotexture.origin = {0.0, 0.0, 0.0};
    infotexture.builder = vsg::Builder::create();

    curan::renderable::Sphere::Info infosphere;
    infosphere.builder = vsg::Builder::create();
    infosphere.geomInfo.color = vsg::vec4(1.0,0.0,0.0,1.0);
    infosphere.geomInfo.dx = vsg::vec3(0.005f,0.0,0.0);
    infosphere.geomInfo.dy = vsg::vec3(0.0,0.005f,0.0);
    infosphere.geomInfo.dz = vsg::vec3(0.0,0.0,0.005f);
    infosphere.stateInfo.blending = true;

    auto mat = vsg::translate(0.0,0.0,0.0);
    
    auto sphere = curan::renderable::Sphere::make(infosphere);
    
    sphere->update_transform(mat);
    window << sphere;

    auto dynamic_texture = curan::renderable::DynamicTexture::make(infotexture);
    window << dynamic_texture;
    auto casted_image = dynamic_texture->cast<curan::renderable::DynamicTexture>();
    auto casted_sphere = sphere->cast<curan::renderable::Sphere>();

    OutputImageType::Pointer output_slice;

    std::atomic<bool> continue_running = true;

    std::thread run_slice_extractor{[&](){
        Eigen::Matrix<double,3,1> needle_tip_in_volume_frame;
        Eigen::Matrix<double,3,1> image_orientation_angles;
        Eigen::Matrix<double,3,1> needle_tip;
        Eigen::Matrix<double,3,3> R_ImageToVolume;
        Eigen::Matrix<double,3,3> R_ImageToWorld;
        double mimic_monotonic_timer = 0.0;
        while (continue_running){
            for (size_t ccc = 0; ccc < 200 && continue_running.load(); ++ccc,mimic_monotonic_timer+=0.005) {
                                             
                std::this_thread::sleep_for(std::chrono::milliseconds(20));

                needle_tip_in_volume_frame[0] = volume_size[0]*volume_spacing[0]*0.5;//-image_size*image_spacing*0.5;
                needle_tip_in_volume_frame[1] = volume_size[1]*volume_spacing[1]*0.5;// -image_size*image_spacing*0.5;//ccc-100.0;
                needle_tip_in_volume_frame[2] = ccc*1.0;

                needle_tip = volume_directionn * needle_tip_in_volume_frame + originn;

                image_orientation_angles[0] = 0.2*std::sin(mimic_monotonic_timer+2.34);//zzz/100.0 - 1.25; 
                image_orientation_angles[1] = 0.2*std::sin(mimic_monotonic_timer+1.234);//zzz/100.0 - 1.25;
                image_orientation_angles[2] = 0.2*std::sin(mimic_monotonic_timer+0.1234);

                obtain_rot_matrix_by_angles(R_ImageToVolume, image_orientation_angles);
                                                
                R_ImageToWorld = volume_directionn * R_ImageToVolume;

                resampler(casted_sphere,casted_image, output_slice, input_volume, needle_tip, R_ImageToWorld, image_size, image_spacing);  

            }
        }
    } 
    };

    window.run();
    continue_running = false;
    run_slice_extractor.join();

    

    return EXIT_SUCCESS;
}