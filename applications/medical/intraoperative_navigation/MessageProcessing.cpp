#include "MessageProcessing.h"
#include <nlohmann/json.hpp>
#include "utils/Reader.h"
#include "robotutils/RobotModel.h"

void obtain_rot_matrix_by_angles(Eigen::Matrix<double, 3, 3> &R_matrix, Eigen::Matrix<double, 3, 1> &rotaton_angles)
{
    typedef double T;

    auto a = rotaton_angles[0];
    auto b = rotaton_angles[1];
    auto c = rotaton_angles[2];

    R_matrix << cos(b) * cos(c), sin(a) * sin(b) * cos(c) - cos(a) * sin(c), cos(a) * sin(b) * cos(c) + sin(a) * sin(c),
        cos(b) * sin(c), sin(a) * sin(b) * sin(c) + cos(a) * cos(c), cos(a) * sin(b) * sin(c) - sin(a) * cos(c),
        -sin(b), sin(a) * cos(b), cos(a) * cos(b);
}

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

template <typename T>
int sgn(T val)
{
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

ProcessingMessage::ProcessingMessage(const ProcessingMessageInfo& info) :
                                        processed_viwer{info.f_in_processed_viwer},
                                        volume{info.f_in_volume},
                                        target{info.f_target},
                                        entry_point{info.f_entry_point},
                                        desired_rotation{info.f_desired_rotation},
                                        registration{info.f_registration},
                                        needle_calibration{info.f_needle_calibration}

{
    shared_pool = curan::utilities::ThreadPool::create(1); 

    auto volume_size = volume->GetLargestPossibleRegion().GetSize();
    auto volume_spacing = volume->GetSpacing();

    image_spacing = std::min(std::min(volume_spacing[0], volume_spacing[1]), volume_spacing[2]);

    Eigen::Matrix<double, 3, 1> volume_size_transformed_to_minimun_spacing;
    volume_size_transformed_to_minimun_spacing[0] = (volume_size[0] * volume_spacing[0]) / image_spacing;
    volume_size_transformed_to_minimun_spacing[1] = (volume_size[1] * volume_spacing[1]) / image_spacing;
    volume_size_transformed_to_minimun_spacing[2] = (volume_size[2] * volume_spacing[2]) / image_spacing;

    image_size = std::sqrt(std::pow(volume_size_transformed_to_minimun_spacing[0], 2) + std::pow(volume_size_transformed_to_minimun_spacing[1], 2) + std::pow(volume_size_transformed_to_minimun_spacing[2], 2));
}

bool ProcessingMessage::process_joint_message(const size_t &protocol_defined_val, const std::error_code &er, std::shared_ptr<curan::communication::FRIMessage> message)
{
    static curan::robotic::RobotModel<7> robot_model{CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json", CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};
    constexpr auto sample_time = std::chrono::milliseconds(1);

    static curan::robotic::State internal_state;
    internal_state.sampleTime = sample_time.count();
    double time = 0;

    internal_state.q = message->angles;

    robot_model.update(internal_state);
    auto robot_to_world = robot_model.homogenenous_transformation();
    robot_to_world(0,3) *= 1e3;
    robot_to_world(1,3) *= 1e3;
    robot_to_world(2,3) *= 1e3;
    auto needle_to_world = robot_to_world * needle_calibration;

    Eigen::Matrix<double, 3, 1> needle_tip = needle_to_world.block<3, 1>(0, 3);
    Eigen::Matrix<double, 3, 3> R_ImageToWorld = needle_to_world.block<3, 3>(0, 0);
    auto world_frame_difference_error = desired_rotation.col(2) - R_ImageToWorld.col(2);

    double phy = world_frame_difference_error.transpose() * desired_rotation.col(0);
    double theta = world_frame_difference_error.transpose() * desired_rotation.col(1);

    OutputImageType::Pointer slice = resampler(volume, needle_tip, R_ImageToWorld, image_size, image_spacing);
    OutputImageType::IndexType coordinate_of_needle_in_image_position;
    OutputImageType::PointType needle_tip_ikt_coordinates{{needle_tip[0], needle_tip[1], needle_tip[2]}};
    slice->TransformPhysicalPointToIndex(needle_tip_ikt_coordinates, coordinate_of_needle_in_image_position);
    OutputImageType::SizeType size_itk = slice->GetLargestPossibleRegion().GetSize();
    auto buff = curan::utilities::CaptureBuffer::make_shared(slice->GetBufferPointer(), slice->GetPixelContainer()->Size() * sizeof(OutputPixelType), slice);
    curan::ui::ImageWrapper wrapper{buff, size_itk[0], size_itk[1]};

    auto inner_projection = R_ImageToWorld.col(2).transpose()*desired_rotation.col(2);
    auto projected_unto_plane = ((needle_tip-target)*R_ImageToWorld.col(2).transpose());
    auto d = (std::abs(inner_projection(0,0))>1e-5) ? projected_unto_plane(0,0)/(inner_projection(0,0)) : 100000.0;

    Eigen::Matrix<double,3,1> real_intersection = target+desired_rotation.col(2)*d;
    OutputImageType::IndexType index_coordinate_of_perfect_traject_in_needle_plane;
    OutputImageType::PointType coordinate_of_perfect_traject_in_needle_plane{{real_intersection[0], real_intersection[1], real_intersection[2]}};
    slice->TransformPhysicalPointToIndex(coordinate_of_perfect_traject_in_needle_plane,index_coordinate_of_perfect_traject_in_needle_plane);

    auto custimized_drawing = [=](SkCanvas *canvas, SkRect image_area, SkRect content_area)
    {
        auto surface_height = canvas->getSurface()->height();
        auto surface_width = canvas->getSurface()->width();

        SkPaint greenPaint;
        greenPaint.setAntiAlias(true);
        greenPaint.setColor(SK_ColorGREEN);

        SkPaint redPaint;
        redPaint.setAntiAlias(true);
        redPaint.setColor(SK_ColorRED);
        int x_image_coordinates = surface_width / 2.0 - index_coordinate_of_perfect_traject_in_needle_plane[0];
        int y_image_coordinates = surface_height / 2.0 - index_coordinate_of_perfect_traject_in_needle_plane[1];
        canvas->drawImage(wrapper.image, x_image_coordinates, y_image_coordinates);
        canvas->drawCircle(SkPoint::Make(x_image_coordinates+coordinate_of_needle_in_image_position[0], y_image_coordinates+coordinate_of_needle_in_image_position[1]), 10, greenPaint); 
        canvas->drawCircle(SkPoint::Make(surface_width / 2.0, surface_height / 2.0), 10, redPaint); 


        Level level;
        double bubble_position = (surface_width-60.0) / 2.0 * ( 1 + theta)+30.0;
        level.draw(canvas,bubble_position,Level::LevelOrientation::horizontal);

        Level levelvert;
        double bubble_position_vert = (surface_height-60.0) / 2.0 * ( 1 + phy)+30.0;
        levelvert.draw(canvas,bubble_position_vert,Level::LevelOrientation::vertical);

        auto error = (target-needle_tip).norm();
        greenPaint.setStroke(true);
        const SkScalar intervals[] = {10.0f, 5.0f, 2.0f, 5.0f};
        size_t count = sizeof(intervals) / sizeof(intervals[0]);
        greenPaint.setPathEffect(SkDashPathEffect::Make(intervals, count, 0.0f));
        canvas->drawCircle(SkPoint::Make(x_image_coordinates+coordinate_of_needle_in_image_position[0], y_image_coordinates+coordinate_of_needle_in_image_position[1]), error, greenPaint); 
        std::cout << "error is: " << error << std::endl;
};
    processed_viwer->update_batch(custimized_drawing, wrapper);
    return false;
}

void ProcessingMessage::communicate()
{
    std::cout << "lauching communication" << std::endl;
    using namespace curan::communication;
    button->set_waiting_color(SK_ColorGREEN);
    io_context.restart();
    asio::ip::tcp::resolver fri_resolver(io_context);
    auto fri_client = curan::communication::Client<curan::communication::protocols::fri>::make(io_context, fri_resolver.resolve("172.31.1.148", std::to_string(50010)));
    auto lam_fri = [&](const size_t &protocol_defined_val, const std::error_code &er, std::shared_ptr<curan::communication::FRIMessage> message){
        try{
          process_joint_message(protocol_defined_val, er, message);      
        } catch (...) {
            std::cout << "Exception \"process_joint_message\" was thrown" << std::endl;
        }
    };
    fri_client->connect(lam_fri);
    std::cout << "io context running" << std::endl;
    connection_status = true;
    io_context.run();
    connection_status = false;
    button->set_waiting_color(SK_ColorRED);
    return;
}

void ProcessingMessage::attempt_stop()
{
    io_context.stop();
}