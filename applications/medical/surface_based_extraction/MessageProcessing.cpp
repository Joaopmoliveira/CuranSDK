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

#include "itkGradientMagnitudeImageFilter.h"

#include "itkCurvatureAnisotropicDiffusionImageFilter.h"
#include "itkGradientMagnitudeRecursiveGaussianImageFilter.h"
#include "itkSigmoidImageFilter.h"
#include "itkFastMarchingImageFilter.h"
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

ImageType::Pointer segment_points(ProcessingMessage *processor, ImageType::Pointer input_image)
{
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
        return nullptr;
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

    //auto reverse_converter = itk::CastImageFilter<itk::Image<float, 2>,ImageType>::New();
    //reverse_converter->SetInput(smoothing->GetOutput());

    auto rescaler = itk::RescaleIntensityImageFilter<ImageType,ImageType>::New();
    rescaler->SetInput(binary_threshold->GetOutput());
    rescaler->SetOutputMinimum(0.0);
    rescaler->SetOutputMaximum(255.0);

    try{
        rescaler->Update();
        return rescaler->GetOutput();
    } catch(...){
        std::cout << "error" << std::endl;
    }

    /*
    using SmoothingFilterType = itk::CurvatureAnisotropicDiffusionImageFilter<itk::Image<float, 2>, itk::Image<float, 2>>;
    auto smoothing = SmoothingFilterType::New();

    using GradientFilterType = itk::GradientMagnitudeRecursiveGaussianImageFilter<itk::Image<float, 2>, itk::Image<float, 2>>;
    using SigmoidFilterType = itk::SigmoidImageFilter<itk::Image<float, 2>, itk::Image<float, 2>>;

    auto gradientMagnitude = GradientFilterType::New();
    smoothing->SetInput(resampleFilter->GetOutput());
    gradientMagnitude->SetInput(smoothing->GetOutput());

    smoothing->SetTimeStep(0.125);
    smoothing->SetNumberOfIterations(5);
    smoothing->SetConductanceParameter(9.0);

    gradientMagnitude->SetSigma(3);

    auto rescaler = itk::RescaleIntensityImageFilter<itk::Image<float, 2>,itk::Image<float, 2>>::New();
    rescaler->SetInput(gradientMagnitude->GetOutput());
    rescaler->SetOutputMinimum(0.0);
    rescaler->SetOutputMaximum(255.0);

    auto reverse_converter = itk::CastImageFilter<itk::Image<float, 2>,ImageType>::New();
    reverse_converter->SetInput(rescaler->GetOutput());
    try{
        reverse_converter->Update();
        return reverse_converter->GetOutput();
    } catch(...){
        std::cout << "error" << std::endl;
    }
        */
    return nullptr;
}

template <typename TImage>
typename TImage::Pointer DeepCopy(typename TImage::Pointer input)
{
    typename TImage::Pointer output = TImage::New();
    output->SetRegions(input->GetLargestPossibleRegion());
    output->SetDirection(input->GetDirection());
    output->SetSpacing(input->GetSpacing());
    output->SetOrigin(input->GetOrigin());
    output->Allocate();

    itk::ImageRegionConstIterator<TImage> inputIterator(input, input->GetLargestPossibleRegion());
    itk::ImageRegionIterator<TImage> outputIterator(output, output->GetLargestPossibleRegion());

    while (!inputIterator.IsAtEnd()){
        outputIterator.Set(inputIterator.Get());
        ++inputIterator;
        ++outputIterator;
    }

    return  output;
}

std::tuple<std::vector<std::pair<unsigned int, unsigned int>>, ImageType::Pointer> segment_points(int min_coordx, int max_coordx, int numLines, ImageType::Pointer input_image)
{

    ImageType::SizeType image_size = input_image->GetLargestPossibleRegion().GetSize();
    double spacing = static_cast<double>(max_coordx - min_coordx) / (numLines - 1);

    std::vector<unsigned int> validXPositions;

    // Cálculo do numero de linhas válidas (com intensidade total maior que 0)
    size_t numValidLines = 0;
    for (int i = 0; i < numLines; ++i)
    {
        unsigned int sumIntensity = 0;
        for (int y = 0; y < image_size[1]; ++y)
        {
            PixelType pixelValue = input_image->GetPixel({{min_coordx + static_cast<int>(i * spacing), y}});
            sumIntensity += pixelValue;
        }
        if (sumIntensity != 0)
        {
            ++numValidLines;
        }
        else
        {
            continue;
        }
        // Guardar posição das linhas válidas
        unsigned int xPosition = min_coordx + static_cast<int>(i * spacing);
        validXPositions.push_back(xPosition);
    }

    if (numValidLines < 1)
        return {std::vector<std::pair<unsigned int, unsigned int>>{}, nullptr};

    // Inicializar vetores e matrizes
    // Matriz que armazena os a coordenadas y de inicio e fim de cada bloco, para cada linha
    std::vector<std::vector<std::pair<unsigned int, unsigned int>>> blockPixelPairs(numValidLines);
    // Matriz que armazena a soma de intensidades de cada bloco, para cada linha (indexado da mesma forma que a matriz anterior)
    std::vector<std::vector<unsigned int>> intensitySums(numValidLines);
    // Vetor que armazena as coordenadas y de inicio e fim do bloco com a soma de intensidades mais alta, para cada linha
    std::vector<std::pair<unsigned int, unsigned int>> highestIntensityPixelPairs(numValidLines);
    // Vetor que armazena a coordanada y do pixel médio do bloco com a soma de intensidades mais alta, para cada linha
    std::vector<unsigned int> midPixel_y(numValidLines);
    // Vetor que armazena os pares de pontos que vão para a regressão
    std::vector<std::pair<unsigned int, unsigned int>> pointsToFit;

    using SignedPixelType = float;
    using SignedImageType = itk::Image<SignedPixelType, 2>;
    using CastFilterType = itk::CastImageFilter<ImageType, SignedImageType>;
    CastFilterType::Pointer castFilter = CastFilterType::New();
    castFilter->SetInput(input_image);

    using GaussianFilterType = itk::SmoothingRecursiveGaussianImageFilter<SignedImageType, SignedImageType>;
    GaussianFilterType::Pointer gaussianFilter = GaussianFilterType::New();
    gaussianFilter->SetInput(castFilter->GetOutput());
    gaussianFilter->SetSigma(1.0);

    gaussianFilter->Update();

    auto converter = itk::CastImageFilter<SignedImageType, ImageType>::New();
    converter->SetInput(gaussianFilter->GetOutput());
    converter->Update();
    SignedImageType::Pointer gaussianImage = gaussianFilter->GetOutput();

    // Iterar para todas as linhas de avaliação
    for (int i = 0; i < numValidLines; ++i)
    {
        // Cálculo da coordanada x para a linha atual
        unsigned int xPosition = validXPositions[i];

        // Inicializar variáveis
        unsigned int maxIntensity = -100;
        bool blockStart = false;
        int blockNum = 0;
        int blockSum = 0;
        unsigned int blockStartPixel;
        unsigned int blockEndPixel;

        // Cálculo da intensidade máxima do pixel para a linha atual
        for (int y = 0; y < image_size[1]; ++y)
        {
            PixelType pixelValue = gaussianImage->GetPixel({{xPosition, y}});
            if (pixelValue > maxIntensity)
            {
                maxIntensity = pixelValue;
            }
        }

        unsigned int threashold = maxIntensity / 2;
        // Iterar na linha, descobrir o número de blocos a considerar, as coordenadas de
        // onde o bloco começa e acaba e calcular a soma de intensidades de cada bloco
        for (int y = 0; y < image_size[1] - 15; ++y)
        {
            PixelType pixelValue = gaussianImage->GetPixel({{xPosition, y}});
            if (pixelValue > threashold)
            {
                if (!blockStart)
                {
                    blockNum++;
                    blockStart = true;
                    blockStartPixel = y;
                }
                blockEndPixel = y;
                blockSum += pixelValue;
            }
            else
            {
                if (blockStart)
                {
                    blockStart = false;
                    blockPixelPairs[i].push_back(std::make_pair(blockStartPixel, blockEndPixel));
                    intensitySums[i].push_back(blockSum);
                    blockSum = 0;
                    // std::cout << "End pixel of Line " << i << ": " << blockEndPixel << std::endl;
                }
            }
        }
        // std::cout << "Num bolcks of Line " << i << ": " << blockNum << std::endl;
    }

    if (blockPixelPairs.size() != numValidLines)
        throw std::runtime_error("blockPixelPairs different from numValidLines");

    for (int i = 0; i < numValidLines; ++i)
    {
        if (blockPixelPairs[i].size() < 1)
            continue;
        int maxYIndex = std::distance(blockPixelPairs[i].begin(), std::max_element(
                                                                      blockPixelPairs[i].begin(), blockPixelPairs[i].end(),
                                                                      [](const std::pair<unsigned int, unsigned int> &a, const std::pair<unsigned int, unsigned int> &b)
                                                                      {
                                                                          return a.second < b.second;
                                                                      }));
        std::pair<unsigned int, unsigned int> highestYBlock = blockPixelPairs[i][maxYIndex];

        midPixel_y[i] = highestYBlock.second;
    }

    for (int i = 0; i < numValidLines; ++i)
    {
        unsigned int xPosition = min_coordx + static_cast<int>(i * spacing);
        unsigned int mid_y = midPixel_y[i];
        pointsToFit.push_back(std::make_pair(xPosition, mid_y));
    }
    return {pointsToFit, converter->GetOutput()};
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

bool codeExecuted = false;
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

    // Frame
    ImageType::Pointer shr_ptr_imported = importFilter->GetOutput();
    auto ptr_imported_copy = DeepCopy<ImageType>(shr_ptr_imported);

    // Pixel x médio
    ImageType::SizeType size_itk = shr_ptr_imported->GetLargestPossibleRegion().GetSize();

    // Usado para definir limites do botão e a janela de análise para o algoritmo de segmentação
    if (!codeExecuted)
    {
        int size_x = size_itk[0] - 1;
        float float_size_x = size_x;
        processor->max_coordx_limit[1] = float_size_x;
        processor->max_coordx_limit[0] = float_size_x - 200.0;
        processor->max_coordx = size_x;

        codeExecuted = true;
    }

    // Segmentation
    auto local_image = segment_points(processor,shr_ptr_imported);
    //auto [local_segmented_points, local_image] = segment_points(processor->min_coordx, processor->max_coordx, processor->numLines, shr_ptr_imported);
    //auto segmented_points = local_segmented_points;
    //auto image_blured = local_image;
    //if (segmented_points.size() == 0)
    //    return true;

    igtl::Matrix4x4 local_mat;
    message_body->GetMatrix(local_mat);

    Eigen::Matrix4d eigen_mat;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            eigen_mat(i, j) = local_mat[i][j];

    ObservationEigenFormat observation_n;

    /*
    if (processor->record_poincloud)
    {
        observation_n.pose = eigen_mat * processor->calibration;
        observation_n.segmented_points = segmented_points;
        processor->list_of_recorded_points.push_back(observation_n);
    }

    if (processor->snapshot)
    {
        std::cout << "snapshotting!\n";
        processor->snapshot = false;
        observation_n.pose = eigen_mat * processor->calibration;
        observation_n.segmented_points = segmented_points;
        processor->list_of_recorded_points.push_back(observation_n);
    }

    if (processor->store_to_file)
    {
        processor->store_to_file = false;
        CreatePointCloud(processor);
    }
    */

    auto buff = curan::utilities::CaptureBuffer::make_shared(ptr_imported_copy->GetBufferPointer(), ptr_imported_copy->GetPixelContainer()->Size() * sizeof(char), ptr_imported_copy);
    curan::ui::ImageWrapper wrapper{buff, size_itk[0], size_itk[1], SkColorType::kGray_8_SkColorType, SkAlphaType::kPremul_SkAlphaType};
    processor->processed_viwer->update_image(wrapper);
    /*
    processor->processed_viwer->update_batch([segmented_points, size_itk, processor](SkCanvas *canvas, SkRect image_area, SkRect widget_area)
                                             { 
        float scalling_factor_x = image_area.width()/size_itk[0];
        float scalling_factor_y = image_area.height()/size_itk[1];
        //Todos os pontos
        SkPaint paint;
        SkColor customColor = SkColorSetARGB(255, 255, 153, 153);
        paint.setColor(customColor); 
        paint.setStyle(SkPaint::kFill_Style); 
        SkScalar radius = 3;
        for (auto& points : segmented_points){
            canvas->drawCircle(points.first * scalling_factor_x + image_area.left(), points.second * scalling_factor_y + image_area.top(), radius, paint);
        } }, wrapper);
    */
    if(local_image.IsNotNull())
    {   
        auto smaller_size_itk = local_image->GetLargestPossibleRegion().GetSize();
        auto buff2 = curan::utilities::CaptureBuffer::make_shared(local_image->GetBufferPointer(), local_image->GetPixelContainer()->Size() * sizeof(char), local_image);
        curan::ui::ImageWrapper wrapper2{buff2, smaller_size_itk[0], smaller_size_itk[1], SkColorType::kGray_8_SkColorType, SkAlphaType::kPremul_SkAlphaType};

        processor->filter_viwer->update_image(wrapper2);

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
