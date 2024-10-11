#include "MessageProcessing.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include "optimization/WireCalibration.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <ctime>
#include <chrono>

#include "itkDerivativeImageFilter.h"
#include "itkSmoothingRecursiveGaussianImageFilter.h"
#include "itkImageRegionIterator.h"
#include "itkCastImageFilter.h"
#include "itkConnectedComponentImageFilter.h"
#include "itkGradientMagnitudeImageFilter.h"
#include "itkMaskImageFilter.h"
#include "itkCurvatureAnisotropicDiffusionImageFilter.h"
#include "itkGradientMagnitudeRecursiveGaussianImageFilter.h"
#include "itkSigmoidImageFilter.h"
#include "itkFastMarchingImageFilter.h"
#include "itkRelabelComponentImageFilter.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkBinomialBlurImageFilter.h"
#include "itkStatisticsImageFilter.h"
#include "itkScalarImageToHistogramGenerator.h"

bool process_transform_message(ProcessingMessage *processor, igtl::MessageBase::Pointer val)
{
    igtl::TransformMessage::Pointer transform_message = igtl::TransformMessage::New();
    transform_message->Copy(val);
    int c = transform_message->Unpack(1);
    if (!(c & igtl::MessageHeader::UNPACK_BODY))
        return false; // failed to unpack message, therefore returning without doing anything
    return true;
}

using PixelType = unsigned char;
constexpr unsigned int Dimension = 2;
using ImageType = itk::Image<PixelType, Dimension>;

template <typename TImage>
typename TImage::Pointer DeepCopy(typename TImage::Pointer input)
{
    typename TImage::Pointer output = TImage::New();
    output->SetRegions(input->GetLargestPossibleRegion());
    output->SetDirection(input->GetDirection());
    output->SetSpacing(input->GetSpacing());
    output->SetOrigin(input->GetOrigin());
    output->Allocate();

    itk::ImageRegionConstIteratorWithIndex<TImage> inputIterator(input, input->GetLargestPossibleRegion());
    itk::ImageRegionIterator<TImage> outputIterator(output, output->GetLargestPossibleRegion());

    while (!inputIterator.IsAtEnd()){
        outputIterator.Set(inputIterator.Get());
        ++inputIterator;
        ++outputIterator;
    }

    return  output;
}

template <typename TImage,typename InclusionPolicy>
typename TImage::Pointer DeepCopyWithInclusionPolicy(InclusionPolicy&& inclusion_policy,typename TImage::Pointer input)
{
    typename TImage::Pointer output = TImage::New();
    output->SetRegions(input->GetLargestPossibleRegion());
    output->SetDirection(input->GetDirection());
    output->SetSpacing(input->GetSpacing());
    output->SetOrigin(input->GetOrigin());
    output->Allocate();

    itk::ImageRegionConstIteratorWithIndex<TImage> inputIterator(input, input->GetLargestPossibleRegion());
    itk::ImageRegionIterator<TImage> outputIterator(output, output->GetLargestPossibleRegion());

    while (!inputIterator.IsAtEnd()){
        if(inclusion_policy((double)(inputIterator.GetIndex()[0]),(double)(inputIterator.GetIndex()[1]))){
            outputIterator.Set(inputIterator.Get());
        }else
            outputIterator.Set(0);
        ++inputIterator;
        ++outputIterator;
    }

    return  output;
}

std::tuple<ImageType::Pointer,std::vector<std::pair<unsigned int,unsigned int>>,std::vector<std::pair<unsigned int,unsigned int>>> segment_points(ProcessingMessage *processor, ImageType::Pointer input_image)
{
    std::vector<std::pair<unsigned int,unsigned int>> found_points;
    std::vector<std::pair<unsigned int,unsigned int>> max_intensity_found_points;
    auto converter = itk::CastImageFilter<ImageType, itk::Image<float, 2>>::New();
    converter->SetInput(input_image);

    auto input_size = input_image->GetLargestPossibleRegion().GetSize();
    auto input_spacing = input_image->GetSpacing();
    auto input_origin = input_image->GetOrigin();

    double physicalspace[2];
    physicalspace[0] = input_size[0] * input_spacing[0];
    physicalspace[1] = input_size[1] * input_spacing[1];

    auto output_size = input_size;
    output_size[0] = (size_t)std::floor((1.0 / 2) * output_size[0]);
    output_size[1] = (size_t)std::floor((1.0 / 2) * output_size[1]);

    auto output_spacing = input_spacing;
    output_spacing[0] = physicalspace[0] / output_size[0];
    output_spacing[1] = physicalspace[1] / output_size[1];

    auto interpolator = itk::LinearInterpolateImageFunction<itk::Image<float, 2>, double>::New();
    auto transform = itk::AffineTransform<double, 2>::New();
    transform->SetIdentity();
    auto resampleFilter = itk::ResampleImageFilter<itk::Image<float, 2>, itk::Image<float, 2>>::New();
    resampleFilter->SetInput(converter->GetOutput());
    resampleFilter->SetTransform(transform);
    resampleFilter->SetInterpolator(interpolator);
    resampleFilter->SetOutputDirection(input_image->GetDirection());
    resampleFilter->SetSize(output_size);
    resampleFilter->SetOutputSpacing(output_spacing);
    resampleFilter->SetOutputOrigin(input_origin);

    auto bluring = itk::BinomialBlurImageFilter<itk::Image<float, 2>, itk::Image<float, 2>>::New();
    bluring->SetInput(resampleFilter->GetOutput());
    bluring->SetRepetitions(4);

    using SmoothingFilterType = itk::CurvatureAnisotropicDiffusionImageFilter<itk::Image<float, 2>, itk::Image<float, 2>>;
    auto smoothing = SmoothingFilterType::New();

    smoothing->SetInput(bluring->GetOutput());
    smoothing->SetTimeStep(processor->timestep);
    smoothing->SetNumberOfIterations(processor->iterations);
    smoothing->SetConductanceParameter(processor->conductance);

    try{
        smoothing->Update();
    } catch(...){
        std::cout << "error" << std::endl;
        return {nullptr,found_points,max_intensity_found_points};
    }

    using HistogramGeneratorType = itk::Statistics::ScalarImageToHistogramGenerator<itk::Image<float, 2>>;
    using HistogramType = HistogramGeneratorType::HistogramType;
    auto histogramGenerator = HistogramGeneratorType::New();
    histogramGenerator->SetInput(smoothing->GetOutput());
    histogramGenerator->SetNumberOfBins(500);
    histogramGenerator->Compute();

    auto histogram = histogramGenerator->GetOutput();
    double total_frequency = 0;
    for (size_t i = 1; i < histogram->Size(); ++i)
        total_frequency += histogram->GetFrequency(i);

    auto target_frequency = processor->percentage * total_frequency;

    double cumulative_frequency = 0;
    size_t threshold_bin = 0;
    for (size_t i = 1; i < histogram->Size(); ++i)
    {
        if (cumulative_frequency >= target_frequency)
        {
            threshold_bin = i;
            break;
        }
        cumulative_frequency += histogram->GetFrequency(i);
    }
    auto minMaxCalculator = itk::MinimumMaximumImageCalculator<itk::Image<float, 2>>::New();
    minMaxCalculator->SetImage(smoothing->GetOutput());
    minMaxCalculator->Compute();

    HistogramType::MeasurementType thresholdvalue = histogram->GetBinMin(0, threshold_bin);

    auto binary_threshold = itk::BinaryThresholdImageFilter<itk::Image<float, 2>, itk::Image<unsigned char, 2>>::New();
    binary_threshold->SetInput(smoothing->GetOutput());
    binary_threshold->SetOutsideValue(0);
    binary_threshold->SetInsideValue(255);
    binary_threshold->SetLowerThreshold(histogram->GetBinMin(0, threshold_bin));
    binary_threshold->SetUpperThreshold(minMaxCalculator->GetMaximum());

    auto connectedComponentFilter = itk::ConnectedComponentImageFilter<ImageType, ImageType>::New();
    connectedComponentFilter->SetInput(binary_threshold->GetOutput());

    auto relabelFilter = itk::RelabelComponentImageFilter<ImageType,ImageType>::New();
    relabelFilter->SetInput(connectedComponentFilter->GetOutput());

    auto filtered_image_with_largest_components = itk::ThresholdImageFilter<ImageType>::New();
    filtered_image_with_largest_components->SetInput(relabelFilter->GetOutput());
    filtered_image_with_largest_components->ThresholdOutside(1,processor->connected_components);
    filtered_image_with_largest_components->SetOutsideValue(255);

    auto final_binary_threshold = itk::BinaryThresholdImageFilter<ImageType,ImageType>::New();
    final_binary_threshold->SetInput(filtered_image_with_largest_components->GetOutput());
    final_binary_threshold->SetOutsideValue(0);
    final_binary_threshold->SetInsideValue(255);
    final_binary_threshold->SetLowerThreshold(0);
    final_binary_threshold->SetUpperThreshold(processor->connected_components);

    try{
        final_binary_threshold->Update();
    } catch (const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << err << std::endl;
        return {nullptr,found_points,max_intensity_found_points};
    } catch (...) {
        std::cout << "generic unknown exception" << std::endl;
        return {nullptr,found_points,max_intensity_found_points};
    }

    auto copied_mask = DeepCopyWithInclusionPolicy<ImageType>([=](double x, double y){ return (y< output_size[1]-5) ? true : false ; },final_binary_threshold->GetOutput());

    using MaskFilterType = itk::MaskImageFilter<itk::Image<float,2>, ImageType>;
    auto maskFilter = MaskFilterType::New();
    maskFilter->SetInput(resampleFilter->GetOutput());
    maskFilter->SetMaskImage(copied_mask);

    auto reconverter_converter = itk::CastImageFilter<itk::Image<float, 2>,ImageType>::New();
    reconverter_converter->SetInput(maskFilter->GetOutput());

    try{
        reconverter_converter->Update();

    } catch (const itk::ExceptionObject &err) {
        std::cout << "ExceptionObject caught !" << err << std::endl;
        return {nullptr,found_points,max_intensity_found_points};
    } catch (...) {
        std::cout << "generic unknown exception" << std::endl;
        return {nullptr,found_points,max_intensity_found_points};
    }

    auto image = reconverter_converter->GetOutput();

    max_intensity_found_points.resize(image->GetLargestPossibleRegion().GetSize()[0]);
    for(int cols = 0; cols < image->GetLargestPossibleRegion().GetSize()[0]; ++cols){
        std::vector<std::tuple<int,int,int>> local_with_max;
        std::vector<std::pair<int,int>> local;
        bool first_found = false;
        int current_max_value = 0;
        int index_of_max = 0;
        for(int rows = 0; rows < image->GetLargestPossibleRegion().GetSize()[1]; ++rows){
            auto current_value = image->GetPixel(itk::Index<2>{cols,rows});
            if(current_value!=0 && !first_found){
                first_found = true;
                current_max_value = 0;
                index_of_max = 0;
            }
            if(current_value==0 && first_found){
                first_found = false;
                local_with_max.emplace_back(cols,index_of_max,current_max_value);
                local.emplace_back(cols,index_of_max);
            }
            if(first_found && current_value>current_max_value){
                current_max_value = current_value;
                index_of_max = rows;
            }
        }
        std::tuple<int,int,int> largest = std::make_tuple<int,int,int>(0,0,0);
        bool found = false;
        for(auto & possible_largest : local_with_max){
            if(std::get<2>(possible_largest)>std::get<2>(largest)){
                largest = possible_largest;
                found = true;
            }
        }
        if(found)
            max_intensity_found_points[cols] = std::make_pair(std::get<0>(largest),std::get<1>(largest));
    }

    found_points.reserve(image->GetLargestPossibleRegion().GetSize()[0]);
    int offset = std::pow(2,processor->horizontal_divisions.load())+1;
    for(size_t col = 0; col < image->GetLargestPossibleRegion().GetSize()[0] ; col+=offset){
        std::vector<std::pair<unsigned int,unsigned int>> vals;
        if(col+offset<image->GetLargestPossibleRegion().GetSize()[0])
            std::copy(max_intensity_found_points.begin()+col,max_intensity_found_points.begin()+col+offset,std::back_inserter(vals));
        else
            break;
        std::sort(vals.begin(),vals.end(),[](std::pair<unsigned int,unsigned int> first, std::pair<unsigned int, unsigned int> second){ return std::get<1>(first) < std::get<1>(second);});
        found_points.emplace_back(vals[std::pow(2,processor->horizontal_divisions.load()-1)]);
    }
    return {image,found_points,max_intensity_found_points};
}

void WritePointCloudToFile(const std::vector<Eigen::Vector3d> &point_cloud, const std::string &filename)
{
    std::cout << "writing to file\n";
    std::ofstream file{filename};
    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return;
    }

    for (const auto &point : point_cloud)
        file << point(0) << " " << point(1) << " " << point(2) << "\n";

    std::cout << "Point cloud written to " << filename << std::endl;
}

int CreatePointCloud(ProcessingMessage *processor)
{
    std::vector<Eigen::Vector3d> point_cloud;
    std::cout << "number of points : " << processor->list_of_recorded_points.size() << std::endl;
    if (processor->list_of_recorded_points.size() < 1)
        return 0;

    for (const auto &observation : processor->list_of_recorded_points)
    {
        for (const auto &segmented_point : observation.segmented_points)
        {
            // Coordenadas da imagem
            unsigned int x = segmented_point.first;
            unsigned int y = segmented_point.second;
            constexpr double pixel_size = 0.00018867924; // confirmar
            Eigen::Vector4d image_point(pixel_size * x, pixel_size * y, 0, 1.0);
            Eigen::Vector4d point_in_world_with_1 = observation.pose * image_point;
            Eigen::Vector3d point_in_world = point_in_world_with_1.head<3>();
            point_cloud.push_back(point_in_world);
        }
    }

    WritePointCloudToFile(point_cloud, "Pointcloud.txt");
    std::cout << "Generated point cloud: " << std::endl;
    return 0;
}

bool process_image_message(ProcessingMessage *processor, igtl::MessageBase::Pointer val)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = begin;
    igtl::ImageMessage::Pointer message_body = igtl::ImageMessage::New();
    message_body->Copy(val);
    int c = message_body->Unpack(1);
    if (!(c & igtl::MessageHeader::UNPACK_BODY))
        return false; // failed to unpack message, therefore returning without doing anything

    int x, y, z;
    igtl::TimeStamp::Pointer ts;
    ts = igtl::TimeStamp::New();
    message_body->GetTimeStamp(ts);

    message_body->GetDimensions(x, y, z);
    using PixelType = unsigned char;
    constexpr unsigned int Dimension = 2;
    using ImageType = itk::Image<PixelType, Dimension>;

    using FloatImageType = itk::Image<float, Dimension>;
    using ImportFilterType = itk::ImportImageFilter<PixelType, Dimension>;
    auto importFilter = ImportFilterType::New();

    ImportFilterType::SizeType size;
    size[0] = x;
    size[1] = y;
    ImportFilterType::IndexType start;
    start.Fill(0);
    ImportFilterType::RegionType region;
    region.SetIndex(start);
    region.SetSize(size);

    importFilter->SetRegion(region);
    const itk::SpacePrecisionType origin[Dimension] = {0.0, 0.0};
    importFilter->SetOrigin(origin);
    const itk::SpacePrecisionType spacing[Dimension] = {1.0, 1.0};
    importFilter->SetSpacing(spacing);

    const bool importImageFilterWillOwnTheBuffer = false;
    importFilter->SetImportPointer((PixelType *)message_body->GetScalarPointer(), message_body->GetScalarSize(), importImageFilterWillOwnTheBuffer);

    try
    {
        importFilter->Update();
    }
    catch (...)
    {
        return false;
    }

    ImageType::Pointer shr_ptr_imported = importFilter->GetOutput();
    auto ptr_imported_copy = DeepCopy<ImageType>(shr_ptr_imported);

    ImageType::SizeType size_itk = shr_ptr_imported->GetLargestPossibleRegion().GetSize();

    auto [minimized_image,array_of_segmented_points,max_array_of_segmented_points] = segment_points(processor,shr_ptr_imported);

    igtl::Matrix4x4 local_mat;
    message_body->GetMatrix(local_mat);

    Eigen::Matrix4d eigen_mat;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            eigen_mat(i, j) = local_mat[i][j];

    ObservationEigenFormat observation_n;

    
    if (processor->record_poincloud)
    {
        observation_n.pose = eigen_mat * processor->calibration;
        observation_n.segmented_points = max_array_of_segmented_points;
        processor->list_of_recorded_points.push_back(observation_n);
    }

    if (processor->snapshot)
    {
        processor->snapshot = false;
        observation_n.pose = eigen_mat * processor->calibration;
        observation_n.segmented_points = max_array_of_segmented_points;
        processor->list_of_recorded_points.push_back(observation_n);
    }

    if (processor->store_to_file)
    {
        processor->store_to_file = false;
        CreatePointCloud(processor);
    }
    

    auto buff = curan::utilities::CaptureBuffer::make_shared(ptr_imported_copy->GetBufferPointer(), ptr_imported_copy->GetPixelContainer()->Size() * sizeof(char), ptr_imported_copy);
    curan::ui::ImageWrapper wrapper{buff, size_itk[0], size_itk[1], SkColorType::kGray_8_SkColorType, SkAlphaType::kPremul_SkAlphaType};
    processor->processed_viwer->update_image(wrapper);
    if(minimized_image.IsNotNull())
    {   
        auto smaller_size_itk = minimized_image->GetLargestPossibleRegion().GetSize();
        auto buff2 = curan::utilities::CaptureBuffer::make_shared(minimized_image->GetBufferPointer(), smaller_size_itk[0]* smaller_size_itk[1] * sizeof(char), minimized_image);
        curan::ui::ImageWrapper wrapper2{buff2, smaller_size_itk[0], smaller_size_itk[1], SkColorType::kGray_8_SkColorType, SkAlphaType::kPremul_SkAlphaType};
        processor->filter_viwer->update_batch([max_array_of_segmented_points,array_of_segmented_points, smaller_size_itk, processor](SkCanvas *canvas, SkRect image_area, SkRect widget_area)
                                             { 
            float scalling_factor_x = image_area.width()/smaller_size_itk[0];
            float scalling_factor_y = image_area.height()/smaller_size_itk[1];
            SkPaint paint;
            paint.setStyle(SkPaint::kFill_Style); 
            SkScalar radius = 6;
            paint.setColor(SK_ColorCYAN); 
            //for(auto& largest : max_array_of_segmented_points)
            //    canvas->drawCircle(largest.first * scalling_factor_x + image_area.left(), largest.second * scalling_factor_y + image_area.top(), radius, paint);
            radius = 3;
            paint.setColor(SK_ColorGREEN);
            for(auto& largest_robust : array_of_segmented_points)
                canvas->drawCircle(largest_robust.first * scalling_factor_x + image_area.left(), largest_robust.second * scalling_factor_y + image_area.top(), radius, paint);        
        }, wrapper2);
    }

    end = std::chrono::steady_clock::now();
    auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    if (time_elapsed > 40)
        std::printf("Warning: Processing time higher than frame to frame interval (%lld milliseconds)\n", time_elapsed);
    return true;
}

std::map<std::string, std::function<bool(ProcessingMessage *, igtl::MessageBase::Pointer)>> openigtlink_callbacks{
    {"TRANSFORM", process_transform_message},
    {"IMAGE", process_image_message}};

bool ProcessingMessage::process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val)
{
    assert(val.IsNotNull());
    if (er)
    {
        return true;
    }
    if (auto search = openigtlink_callbacks.find(val->GetMessageType()); search != openigtlink_callbacks.end())
        search->second(this, val);
    else
        std::cout << "No functionality for function received\n";
    return false;
}

void ProcessingMessage::communicate()
{
    try
    {
        list_of_recorded_points.clear();
        using namespace curan::communication;
        button->set_waiting_color(SK_ColorGREEN);
        io_context.reset();
        asio::ip::tcp::resolver resolver(io_context);
        auto client = Client<curan::communication::protocols::igtlink>::make(io_context, resolver.resolve("localhost", std::to_string(port)));
        connection_status.set(true);
        auto lam = [this](size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val)
        {
            try
            {
                if (process_message(protocol_defined_val, er, val))
                {
                    attempt_stop();
                }
            }
            catch (...)
            {
                std::cout << "Exception was thrown\n";
            }
        };
        client->connect(lam);
        io_context.run();
        connection_status.set(false);
        button->set_waiting_color(SK_ColorRED);
        return;
    }
    catch (...)
    {
        std::cout << "Exception in communicate was thrown\n";
    }
}

void ProcessingMessage::attempt_stop()
{
    io_context.stop();
}
