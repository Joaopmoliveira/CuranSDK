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
#include "itkIntensityWindowingImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkMinimumMaximumImageCalculator.h"
#include "itkExtractImageFilter.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkScaleTransform.h"
#include "itkAffineTransform.h"
#include "itkEuler3DTransform.h"

#include <optional>
#include <chrono>
#include <thread>
#include "Mathematics/IntrPlane3OrientedBox3.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include "itkImage.h"
#include "itkImageFileReader.h"

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

void exclude_row_matrix(Eigen::MatrixXd &matrix, size_t row_to_remove)
{
    int numRows = static_cast<int>(matrix.rows()) - 1;
    int numCols = static_cast<int>(matrix.cols());

    if (row_to_remove < numRows)
    {
        matrix.block(row_to_remove, 0, numRows - row_to_remove, numCols) = matrix.bottomRows(numRows - row_to_remove);
    }

    matrix.conservativeResize(numRows, numCols);
}

Eigen::MatrixXd calculate_image_centroid(const Eigen::Matrix<double, 3, 1> &volume_size_mm, Eigen::Matrix<double, 3, 1> &centroid, const Eigen::Matrix<double, 3, 3> &R_ImageToVolume, const Eigen::Matrix<double, 3, 1> &needle_tip_transformed_to_volume_space)
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
        if (std::abs(vol_vertices_coords(j, 0)) < allowed_error)
        {
            double majored_value = ((std::abs(plane_equation[0]) >= allowed_error) ? plane_equation[0] : allowed_error);
            intersections(i, 0) = -(plane_equation[1] * vol_vertices_coords(j, 1) + plane_equation[2] * vol_vertices_coords(j, 2) + plane_equation[3]) / majored_value;
            intersections(i, 1) = vol_vertices_coords(j, 1);
            intersections(i, 2) = vol_vertices_coords(j, 2);
            i = i + 1;
        }
        if (std::abs(vol_vertices_coords(j, 1)) < allowed_error)
        {
            double majored_value = ((std::abs(plane_equation[1]) >= allowed_error) ? plane_equation[1] : allowed_error);
            intersections(i, 1) = -(plane_equation[0] * vol_vertices_coords(j, 0) + plane_equation[2] * vol_vertices_coords(j, 2) + plane_equation[3]) / majored_value;
            intersections(i, 0) = vol_vertices_coords(j, 0);
            intersections(i, 2) = vol_vertices_coords(j, 2);
            i = i + 1;
        }

        if (std::abs(vol_vertices_coords(j, 2)) < allowed_error)
        {
            double majored_value = ((std::abs(plane_equation[2]) >= allowed_error) ? plane_equation[2] : allowed_error);
            intersections(i, 2) = -(plane_equation[1] * vol_vertices_coords(j, 1) + plane_equation[0] * vol_vertices_coords(j, 0) + plane_equation[3]) / majored_value;
            intersections(i, 0) = vol_vertices_coords(j, 0);
            intersections(i, 1) = vol_vertices_coords(j, 1);
            i = i + 1;
        }
    }

    size_t initial_number_of_points = static_cast<int>(intersections.rows()) - 1;
    bool no_rows = false;
    for (int i = initial_number_of_points; i >= 0; --i)
    {
        if (intersections(i, 0) > volume_size_mm[0] + allowed_error || intersections(i, 0) < 0.0 - allowed_error || intersections(i, 1) > volume_size_mm[1] + allowed_error || intersections(i, 1) < 0.0 - allowed_error || intersections(i, 2) > volume_size_mm[2] + allowed_error || intersections(i, 2) < 0.0 - allowed_error)
        {
            if (intersections.rows() == 1)
            {
                no_rows = true;
                break;
            }
            exclude_row_matrix(intersections, i);
        }
    }

    centroid = (no_rows) ? needle_tip_transformed_to_volume_space : intersections.colwise().mean();
    return intersections;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

OutputImageType::Pointer resampler(InputImageType::Pointer volume, Eigen::Matrix<double, 3, 1> &needle_tip, Eigen::Matrix<double, 3, 3> &R_ImageToWorld, double &image_size, double &image_spacing)
{
    using FilterType = itk::ResampleImageFilter<InputImageType, OutputImageType>;
    auto filter = FilterType::New();

    using InterpolatorType = itk::LinearInterpolateImageFunction<InputImageType, double>;
    // using InterpolatorType = itk::NearestNeighborInterpolateImageFunction<InputImageType, double>;
    auto interpolator = InterpolatorType::New();
    filter->SetInterpolator(interpolator);
    filter->SetDefaultPixelValue(0);

    filter->SetInput(volume);

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

    auto volume_origin = volume->GetOrigin();
    Eigen::Matrix<double, 3, 1> volume_originn;
    volume_originn[0] = volume_origin[0];
    volume_originn[1] = volume_origin[1];
    volume_originn[2] = volume_origin[2];

    auto volume_direction = volume->GetDirection();
    Eigen::Matrix<double, 3, 3> R_volumeToWorld;
    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            R_volumeToWorld(col, row) = volume_direction(col, row);

    // obtain needle tip coordinates in the volume reference frame
    Eigen::Matrix<double, 3, 1> needle_tip_transformed_to_volume_space = R_volumeToWorld.transpose() * needle_tip - R_volumeToWorld.transpose() * volume_originn;
    Eigen::Matrix<double, 3, 3> R_ImageToVolume = R_volumeToWorld.transpose() * R_ImageToWorld;
    Eigen::Matrix<double, 3, 1> centroid;

    auto volume_size = volume->GetLargestPossibleRegion().GetSize();
    auto volume_spacing = volume->GetSpacing();

    Eigen::Matrix<double, 3, 1> volume_size_mm;
    volume_size_mm[0] = volume_size.GetSize()[0] * volume_spacing[0];
    volume_size_mm[1] = volume_size.GetSize()[1] * volume_spacing[1];
    volume_size_mm[2] = volume_size.GetSize()[2] * volume_spacing[2];
    auto intersection_verticies = calculate_image_centroid(volume_size_mm, centroid, R_ImageToVolume, needle_tip_transformed_to_volume_space);

    Eigen::Matrix<double, 3, 1> origin_to_centroid_image_vector;
    origin_to_centroid_image_vector[0] = image_size * image_spacing * 0.5;
    origin_to_centroid_image_vector[1] = image_size * image_spacing * 0.5;
    origin_to_centroid_image_vector[2] = 0.0;

    Eigen::Matrix<double, 3, 1> centroid_in_word_space = R_volumeToWorld * centroid + volume_originn;

    Eigen::Matrix<double, 3, 1> image_origin_vol_ref;
    image_origin_vol_ref = centroid - R_ImageToVolume * origin_to_centroid_image_vector;

    Eigen::Matrix<double, 3, 1> image_origin_world_ref;
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

    return filter->GetOutput();
}

struct Level
{
    enum LevelOrientation
    {
        vertical,
        horizontal
    };

    void draw(SkCanvas *canvas, int bubble_position, LevelOrientation orientation)
    {
        auto surface_height = canvas->getSurface()->height();
        auto surface_width = canvas->getSurface()->width();
        SkPaint greenPaint;
        greenPaint.setAntiAlias(true);
        greenPaint.setColor(SK_ColorGREEN);
        switch (orientation)
        {
        case LevelOrientation::horizontal:
        {
            SkPaint level_horizontal;
            SkPoint points_level[2] = {SkPoint::Make(0.0, 0.0),
                                       SkPoint::Make(0.0, 60.0)};
            SkColor colors_level[2] = {SkColorSetARGB(255.0, 1.0, 65.0, 0.0),
                                       SkColorSetARGB(255.0, 114.0, 213.0, 0.0)};
            level_horizontal.setShader(SkGradientShader::MakeLinear(points_level, colors_level, NULL, 2, SkTileMode::kClamp, 0, NULL));
            SkPath path_level;
            path_level.moveTo(60, 0);             // coner 1
            path_level.lineTo(surface_width-60, 0);  // coner 2
            path_level.lineTo(surface_width-60, 60); // coner 3
            path_level.lineTo(60, 60);            // coner 4
            canvas->drawPath(path_level, level_horizontal);

            SkPaint level_bubble;
            level_bubble.setAntiAlias(true);
            SkColor colors_buble[2] = {SkColorSetARGB(255.0, 148.0, 245.0, 0.0),
                                       SkColorSetARGB(255.0, 180.0, 200.0, 0.0)};
            SkColor colors[2] = {SkColorSetARGB(255.0, 148.0, 245.0, 0.0),
                                 SkColorSetARGB(255.0, 250.0, 250.0, 250.0)};

            level_bubble.setShader(SkShaders::Blend(
                SkBlendMode::kSoftLight, SkGradientShader::MakeTwoPointConical(SkPoint::Make(bubble_position, 30.0), 30.0f, SkPoint::Make(bubble_position, 6.0), 2.0f, colors, nullptr, 2, SkTileMode::kClamp, 0, nullptr), SkGradientShader::MakeRadial(SkPoint::Make(bubble_position, 30.0), 15.0, colors_buble, NULL, 2, SkTileMode::kClamp)));

            canvas->drawOval(SkRect::MakeXYWH(bubble_position - 30, 10.0, 50, 40), level_bubble);
            SkPaint black_lines;
            black_lines.setAntiAlias(true);
            black_lines.setColor(SK_ColorBLACK);
            black_lines.setStroke(true);
            black_lines.setStrokeWidth(3);
            canvas->drawLine(SkPoint::Make(surface_width / 2.0 - 90, 0.0), SkPoint::Make(surface_width / 2.0 - 90, 60.0), black_lines);
            canvas->drawLine(SkPoint::Make(surface_width / 2.0 + 90, 0.0), SkPoint::Make(surface_width / 2.0 + 90, 60.0), black_lines);
        }
        break;
        case LevelOrientation::vertical:
        {
            SkPaint level_vertical;
            SkPoint points_level[2] = {SkPoint::Make(0.0, 0.0),
                                       SkPoint::Make(60.0, 0.0)};
            SkColor colors_level[2] = {SkColorSetARGB(255.0, 1.0, 65.0, 0.0),
                                       SkColorSetARGB(255.0, 114.0, 213.0, 0.0)};
            level_vertical.setShader(SkGradientShader::MakeLinear(points_level, colors_level, NULL, 2, SkTileMode::kClamp, 0, NULL));
            SkPath path_level;
            path_level.moveTo(0, 60);             // coner 1
            path_level.lineTo(60, 60);  // coner 2
            path_level.lineTo(60, surface_height-60); // coner 3
            path_level.lineTo(0, surface_height-60);            // coner 4
            canvas->drawPath(path_level, level_vertical);

            SkPaint level_bubble;
            level_bubble.setAntiAlias(true);
            SkColor colors_buble[2] = {SkColorSetARGB(255.0, 148.0, 245.0, 0.0),
                                       SkColorSetARGB(255.0, 180.0, 200.0, 0.0)};
            SkColor colors[2] = {SkColorSetARGB(255.0, 148.0, 245.0, 0.0),
                                 SkColorSetARGB(255.0, 250.0, 250.0, 250.0)};

            level_bubble.setShader(SkShaders::Blend(
                SkBlendMode::kSoftLight, SkGradientShader::MakeTwoPointConical(SkPoint::Make(30.0,bubble_position), 30.0f, SkPoint::Make(6.0,bubble_position), 2.0f, colors, nullptr, 2, SkTileMode::kClamp, 0, nullptr), SkGradientShader::MakeRadial(SkPoint::Make(30.0,bubble_position), 15.0, colors_buble, NULL, 2, SkTileMode::kClamp)));

            canvas->drawOval(SkRect::MakeXYWH(10.0,bubble_position - 30, 40, 50), level_bubble);
            SkPaint black_lines;
            black_lines.setAntiAlias(true);
            black_lines.setColor(SK_ColorBLACK);
            black_lines.setStroke(true);
            black_lines.setStrokeWidth(3);
            canvas->drawLine(SkPoint::Make(0.0,surface_height / 2.0 - 90), SkPoint::Make(60.0,surface_height / 2.0 - 90), black_lines);
            canvas->drawLine(SkPoint::Make(0.0,surface_height / 2.0 + 90), SkPoint::Make(60.0,surface_height / 2.0 + 90), black_lines);
        }
            break;
        }
    }
};

int main()
{
    try
    {
        nlohmann::json trajectory_data;
	    std::ifstream in(CURAN_COPIED_RESOURCE_PATH"/trajectory_specification.json");
	    in >> trajectory_data;


        std::string timestamp = trajectory_data["timestamp"];
    	std::string target = trajectory_data["target"];
	    std::string entry = trajectory_data["entry"];
	    
        Eigen::Matrix<double, 3, 1> desired_target_point;
        Eigen::Matrix<double, 3, 3> desired_direction;

        using namespace curan::ui;
        std::unique_ptr<Context> context = std::make_unique<Context>();
        ;
        DisplayParams param{std::move(context), 1800, 1000};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

        InputImageType::Pointer input_volume = load_dicom();
        if (input_volume.GetPointer() == nullptr)
            return 1;

        auto volume_size = input_volume->GetLargestPossibleRegion().GetSize();
        auto volume_spacing = input_volume->GetSpacing();

        double image_spacing = std::min(std::min(volume_spacing[0], volume_spacing[1]), volume_spacing[2]);

        auto direction = input_volume->GetDirection();
        auto origin = input_volume->GetOrigin();

        Eigen::Matrix<double, 3, 1> originn = {origin[0], origin[1], origin[2]};
        Eigen::Matrix<double, 3, 3> volume_directionn;
        for (size_t row = 0; row < 3; ++row)
            for (size_t col = 0; col < 3; ++col)
                volume_directionn(col, row) = direction(row, col);

        desired_direction = volume_directionn;
        long index_x = std::ceil(volume_size[0] / 2.0);
        long index_y = std::ceil(volume_size[1] / 2.0);
        long index_z = std::ceil(volume_size[2] / 2.0);
        InputImageType::IndexType local_index{{index_x,index_y , index_z}};
        InputImageType::PointType center_of_volume;
        input_volume->TransformIndexToPhysicalPoint(local_index, center_of_volume);

        desired_target_point[0] = center_of_volume[0];
        desired_target_point[1] = center_of_volume[1];
        desired_target_point[2] = center_of_volume[2];

        Eigen::Matrix<double, 3, 1> volume_size_transformed_to_minimun_spacing;
        volume_size_transformed_to_minimun_spacing[0] = (volume_size[0] * volume_spacing[0]) / image_spacing;
        volume_size_transformed_to_minimun_spacing[1] = (volume_size[1] * volume_spacing[1]) / image_spacing;
        volume_size_transformed_to_minimun_spacing[2] = (volume_size[2] * volume_spacing[2]) / image_spacing;

        double image_size = std::sqrt(std::pow(volume_size_transformed_to_minimun_spacing[0], 2) + std::pow(volume_size_transformed_to_minimun_spacing[1], 2) + std::pow(volume_size_transformed_to_minimun_spacing[2], 2));

        Eigen::Matrix<double, 3, 1> needle_tip_in_volume_frame;
        Eigen::Matrix<double, 3, 1> image_orientation_angles;
        Eigen::Matrix<double, 3, 1> needle_tip;
        Eigen::Matrix<double, 3, 3> R_ImageToVolume;
        Eigen::Matrix<double, 3, 3> R_ImageToWorld;
        double mimic_monotonic_timer = 0.0;
        size_t ccc = 0;

        while (!glfwWindowShouldClose(viewer->window))
        {
            ++ccc;
            mimic_monotonic_timer += 0.01;
            auto start = std::chrono::high_resolution_clock::now();
            SkSurface *pointer_to_surface = viewer->getBackbufferSurface();
            SkCanvas *canvas = pointer_to_surface->getCanvas();

            canvas->clear(SK_ColorBLACK);
            double translation_scalling_factor = 0.5;
            needle_tip_in_volume_frame[0] = volume_size[0] * volume_spacing[0] * 0.5+translation_scalling_factor*volume_size[0] * volume_spacing[0]*std::sin(mimic_monotonic_timer + 2.34); //-image_size*image_spacing*0.5;
            needle_tip_in_volume_frame[1] = volume_size[1] * volume_spacing[1] * 0.5+translation_scalling_factor*volume_size[1] * volume_spacing[1]*std::sin(mimic_monotonic_timer + 1.34); // -image_size*image_spacing*0.5;//ccc-100.0;
            needle_tip_in_volume_frame[2] = volume_size[2] * volume_spacing[2] * (ccc % 200)/200.0;

            needle_tip = volume_directionn * needle_tip_in_volume_frame + originn;

            double orientation_scalling_factor = 0.3;
            image_orientation_angles[0] = orientation_scalling_factor * std::sin(mimic_monotonic_timer + 2.34);  // zzz/100.0 - 1.25;
            image_orientation_angles[1] = orientation_scalling_factor * std::sin(mimic_monotonic_timer + 1.234); // zzz/100.0 - 1.25;
            image_orientation_angles[2] = orientation_scalling_factor * std::sin(mimic_monotonic_timer + 0.1234);

            obtain_rot_matrix_by_angles(R_ImageToVolume, image_orientation_angles);

            R_ImageToWorld = volume_directionn * R_ImageToVolume;

            auto world_frame_difference_error = desired_direction.col(2)-R_ImageToWorld.col(2);

            double phy = world_frame_difference_error.transpose()*desired_direction.col(0);
            double theta = world_frame_difference_error.transpose()*desired_direction.col(1);
            
            OutputImageType::Pointer slice = resampler(input_volume, needle_tip, R_ImageToWorld, image_size, image_spacing);
            OutputImageType::IndexType coordinate_of_needle_in_image_position;
            OutputImageType::PointType needle_tip_ikt_coordinates{{needle_tip[0], needle_tip[1], needle_tip[2]}};
            slice->TransformPhysicalPointToIndex(needle_tip_ikt_coordinates, coordinate_of_needle_in_image_position);
            OutputImageType::SizeType size_itk = slice->GetLargestPossibleRegion().GetSize();
            auto buff = curan::utilities::CaptureBuffer::make_shared(slice->GetBufferPointer(), slice->GetPixelContainer()->Size() * sizeof(OutputPixelType), slice);
            curan::ui::ImageWrapper wrapper{buff, size_itk[0], size_itk[1]};

            auto inner_projection = R_ImageToWorld.col(2).transpose()*desired_direction.col(2);
            auto projected_unto_plane = ((needle_tip-desired_target_point)*R_ImageToWorld.col(2).transpose());
            auto d = (std::abs(inner_projection(0,0))>1e-5) ? projected_unto_plane(0,0)/(inner_projection(0,0)) : 100000.0;

            Eigen::Matrix<double,3,1> real_intersection = desired_target_point+desired_direction.col(2)*d;
            OutputImageType::IndexType index_coordinate_of_perfect_traject_in_needle_plane;
            OutputImageType::PointType coordinate_of_perfect_traject_in_needle_plane{{real_intersection[0], real_intersection[1], real_intersection[2]}};
            slice->TransformPhysicalPointToIndex(coordinate_of_perfect_traject_in_needle_plane,index_coordinate_of_perfect_traject_in_needle_plane);

            auto surface_height = canvas->getSurface()->height();
            auto surface_width = canvas->getSurface()->width();

            SkPaint greenPaint;
            greenPaint.setAntiAlias(true);
            greenPaint.setColor(SK_ColorGREEN);

            SkPaint redPaint;
            redPaint.setAntiAlias(true);
            redPaint.setColor(SK_ColorRED);

            canvas->drawImage(wrapper.image, surface_width / 2.0 - coordinate_of_needle_in_image_position[0], surface_height / 2.0 - coordinate_of_needle_in_image_position[1]);
            canvas->drawCircle(SkPoint::Make(surface_width / 2.0, surface_height / 2.0), 10, greenPaint); 

            Level level;
            double bubble_position = (surface_width-60.0) / 2.0 * ( 1 + theta)+30.0;
            level.draw(canvas,bubble_position,Level::LevelOrientation::horizontal);

            Level levelvert;
            double bubble_position_vert = (surface_height-60.0) / 2.0 * ( 1 + phy)+30.0;
            levelvert.draw(canvas,bubble_position_vert,Level::LevelOrientation::vertical);

            auto error = (desired_target_point-needle_tip).norm();
            greenPaint.setStroke(true);
            const SkScalar intervals[] = {10.0f, 5.0f, 2.0f, 5.0f};
            size_t count = sizeof(intervals) / sizeof(intervals[0]);
            greenPaint.setPathEffect(SkDashPathEffect::Make(intervals, count, 0.0f));
            canvas->drawCircle(SkPoint::Make(surface_width / 2.0, surface_height / 2.0), error, greenPaint); 

            canvas->drawCircle(SkPoint::Make(surface_width / 2.0 - coordinate_of_needle_in_image_position[0]+index_coordinate_of_perfect_traject_in_needle_plane[0], surface_height / 2.0 - coordinate_of_needle_in_image_position[1]+index_coordinate_of_perfect_traject_in_needle_plane[1]), 10, redPaint); 

            auto signals = viewer->process_pending_signals();

            glfwPollEvents();

            bool val = viewer->swapBuffers();
            if (!val)
                std::cout << "failed to swap buffers\n";
            auto end = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
        }
        return 0;
    }
    catch (std::exception &e)
    {
        std::cout << "Failed: " << e.what() << std::endl;
        return 1;
    }
}