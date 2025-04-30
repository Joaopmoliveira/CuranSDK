#include "MessageProcessing.h"
#include <nlohmann/json.hpp>
#include "utils/Reader.h"
#include "robotutils/RobotModel.h"

OutputImageType::Pointer resampler(const InputImageType::Pointer volume,const Eigen::Matrix<double, 3, 1> &target, const Eigen::Matrix<double, 3, 1> &needle_tip,const Eigen::Matrix<double, 3, 3> &R_ImageToWorld, double &image_size, double &image_spacing)
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

    Eigen::Vector4d origin_corner_image_frame = Eigen::Vector4d::Ones();
    origin_corner_image_frame[0] = -image_spacing*image_size*0.5;
    origin_corner_image_frame[1] = -image_spacing*image_size*0.5;
    origin_corner_image_frame[2] = (needle_tip-target).transpose()*R_ImageToWorld.col(2);

    Eigen::Matrix<double,4,4> image_frame_transform = Eigen::Matrix<double,4,4>::Identity();
    image_frame_transform.block<3,3>(0,0) = R_ImageToWorld;
    image_frame_transform(0,3) = target[0];
    image_frame_transform(1,3) = target[1];
    image_frame_transform(2,3) = target[2];

    Eigen::Vector4d origin_corner_world_frame = image_frame_transform*origin_corner_image_frame;

    itk::Point<double, 3> origin;
    origin[0] = origin_corner_world_frame[0];
    origin[1] = origin_corner_world_frame[1];
    origin[2] = origin_corner_world_frame[2];
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
                                        desired_rotation{info.f_desired_rotation},
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

    OutputImageType::Pointer slice = resampler(volume, target , needle_tip, R_ImageToWorld, image_size, image_spacing);
    OutputImageType::SizeType size_itk = slice->GetLargestPossibleRegion().GetSize();
    auto buff = curan::utilities::CaptureBuffer::make_shared(slice->GetBufferPointer(), slice->GetPixelContainer()->Size() * sizeof(OutputPixelType), slice);
    curan::ui::ImageWrapper wrapper{buff, size_itk[0], size_itk[1]};

    OutputImageType::IndexType index_coordinate_of_perfect_traject_in_needle_plane;
    OutputImageType::PointType coordinate_of_perfect_traject_in_needle_plane{{needle_tip[0], needle_tip[1], needle_tip[2]}};
    slice->TransformPhysicalPointToIndex(coordinate_of_perfect_traject_in_needle_plane,index_coordinate_of_perfect_traject_in_needle_plane);

    std::printf("[%d %d %d]\n",(int)index_coordinate_of_perfect_traject_in_needle_plane[0],(int)index_coordinate_of_perfect_traject_in_needle_plane[1],(int)index_coordinate_of_perfect_traject_in_needle_plane[2]);

    auto world_frame_difference_error = desired_rotation.col(2) - R_ImageToWorld.col(2);
    double phy = world_frame_difference_error.transpose() * desired_rotation.col(0);
    double theta = world_frame_difference_error.transpose() * desired_rotation.col(1);

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
        canvas->drawImage(wrapper.image, image_area.x(), image_area.y());
        canvas->drawCircle(SkPoint::Make(image_area.x()+index_coordinate_of_perfect_traject_in_needle_plane[0], image_area.y()+index_coordinate_of_perfect_traject_in_needle_plane[1]), 10, greenPaint); 
        canvas->drawCircle(SkPoint::Make(image_area.centerX(), image_area.centerY()), 10, redPaint); 

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
        canvas->drawCircle(SkPoint::Make(image_area.x()+index_coordinate_of_perfect_traject_in_needle_plane[0], image_area.y()+index_coordinate_of_perfect_traject_in_needle_plane[1]), error, greenPaint); 
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