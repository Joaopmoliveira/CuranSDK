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
#include "utils/Reader.h"
#include <nlohmann/json.hpp>

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
/*
// Funçao que segmenta os pontos
std::vector<std::pair<unsigned int, unsigned int>> segment_points(int min_coordx, int max_coordx, int numLines, ImageType::Pointer input_image)
{

    // Dimensões da imagem
    ImageType::SizeType image_size = input_image->GetLargestPossibleRegion().GetSize();
    // std::cout << "Image Dimensions: "
    //   << image_size[0] << " x " << image_size[1] << std::endl;

    // Cálculo do espaçamento entre linhas
    double spacing = static_cast<double>(max_coordx - min_coordx) / (numLines - 1);

    // Vetor para guardar linhas válidas
    std::vector<unsigned int> validXPositions;

    // Cálculo do numero de linhas válidas (com intensidade total maior que 0)
    int numValidLines = 0;
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

    using SignedPixelType = float;
    using SignedImageType = itk::Image<SignedPixelType, 2>;
    using CastFilterType = itk::CastImageFilter<ImageType, SignedImageType>;
    CastFilterType::Pointer castFilter = CastFilterType::New();
    castFilter->SetInput(input_image);

    using GaussianFilterType = itk::SmoothingRecursiveGaussianImageFilter<SignedImageType, SignedImageType>;
    GaussianFilterType::Pointer gaussianFilter = GaussianFilterType::New();
    gaussianFilter->SetInput(castFilter->GetOutput());
    gaussianFilter->SetSigma(2.0);
    gaussianFilter->Update();

    using DerivativeFilterType = itk::DerivativeImageFilter<SignedImageType, SignedImageType>;
    DerivativeFilterType::Pointer derivativeFilter = DerivativeFilterType::New();
    derivativeFilter->SetInput(castFilter->GetOutput());
    derivativeFilter->SetDirection(1); // Y-direction gradient
    derivativeFilter->SetOrder(1);     // First-order derivative
    derivativeFilter->Update();

    SignedImageType::Pointer gradientImage = derivativeFilter->GetOutput();

    std::vector<std::pair<unsigned int, unsigned int>> maxGradientCoordinates;
    for (auto &xPosition : validXPositions)
    {
        unsigned int maxGradientY = 0;
        PixelType maxGradientValue = std::numeric_limits<PixelType>::lowest();
        for (unsigned int y = 0; y < image_size[1]; ++y)
        {
            PixelType gradientValue = gradientImage->GetPixel({{xPosition, y}});
            if (gradientValue > maxGradientValue)
            {
                maxGradientValue = gradientValue;
                maxGradientY = y;
            }
        }

        maxGradientCoordinates.push_back({xPosition, maxGradientY});
    }

    
    for (const auto& coord : maxGradientCoordinates) {
        std::cout << "X: " << coord.first << ", Y: " << coord.second << std::endl;
    }

    return maxGradientCoordinates;
}
*/

std::vector<std::pair<unsigned int, unsigned int>> segment_points(int min_coordx, int max_coordx, int numLines, ImageType::Pointer input_image){

    //Dimensões da imagem
    ImageType::SizeType image_size = input_image->GetLargestPossibleRegion().GetSize();
    //std::cout << "Image Dimensions: "
            //  << image_size[0] << " x " << image_size[1] << std::endl;

    //Cálculo do espaçamento entre linhas
    double spacing = static_cast<double>(max_coordx - min_coordx) / (numLines-1);

    // Vetor para guardar linhas válidas
    std::vector<unsigned int> validXPositions;

    // Cálculo do numero de linhas válidas (com intensidade total maior que 0)
    int numValidLines = 0;
    for (int i = 0; i < numLines; ++i) {
        unsigned int sumIntensity = 0;
        for (int y = 0; y < image_size[1]; ++y) {
            PixelType pixelValue = input_image->GetPixel({{min_coordx + static_cast<int>(i * spacing), y}});
            sumIntensity += pixelValue;
        }
        if (sumIntensity != 0) {
            ++numValidLines;
        }else{
            continue;
        }
        //Guardar posição das linhas válidas
        unsigned int xPosition = min_coordx + static_cast<int>(i * spacing);
        validXPositions.push_back(xPosition);
    }
    /*
    std::cout << numValidLines << std::endl;
    std::cout << "Valid X Positions: ";
    for (unsigned int x : validXPositions) {
        std::cout << x << " ";
    }
    std::cout << std::endl;
*/
    //Inicializar vetores e matrizes
    //Matriz que armazena os a coordenadas y de inicio e fim de cada bloco, para cada linha
    std::vector<std::vector<std::pair<unsigned int, unsigned int>>> blockPixelPairs(numValidLines);
    //Matriz que armazena a soma de intensidades de cada bloco, para cada linha (indexado da mesma forma que a matriz anterior)
    std::vector<std::vector<unsigned int>> intensitySums(numValidLines);
    //Vetor que armazena as coordenadas y de inicio e fim do bloco com a soma de intensidades mais alta, para cada linha
    std::vector<std::pair<unsigned int, unsigned int>> highestIntensityPixelPairs(numValidLines);
    //Vetor que armazena a coordanada y do pixel médio do bloco com a soma de intensidades mais alta, para cada linha
    std::vector<unsigned int> midPixel_y(numValidLines);
    //Vetor que armazena os pares de pontos que vão para a regressão
    std::vector<std::pair<unsigned int, unsigned int>> pointsToFit;



    using SignedPixelType = float;
    using SignedImageType = itk::Image<SignedPixelType, 2>;
    using CastFilterType = itk::CastImageFilter<ImageType, SignedImageType>;
    CastFilterType::Pointer castFilter = CastFilterType::New();
    castFilter->SetInput(input_image);

    using GaussianFilterType = itk::SmoothingRecursiveGaussianImageFilter<SignedImageType, SignedImageType>;
    GaussianFilterType::Pointer gaussianFilter = GaussianFilterType::New();
    gaussianFilter->SetInput(castFilter->GetOutput());
    gaussianFilter->SetSigma(2.0);
    gaussianFilter->Update();

        SignedImageType::Pointer gaussianImage = gaussianFilter->GetOutput();



    //Iterar para todas as linhas de avaliação
    for (int i = 0; i < numValidLines; ++i) {
        //Cálculo da coordanada x para a linha atual
        unsigned int xPosition = validXPositions[i];

        //Inicializar variáveis
        unsigned int maxIntensity = 0;
        bool blockStart = false;
        int blockNum = 0;
        int blockSum = 0;
        unsigned int blockStartPixel;
        unsigned int blockEndPixel;

        //Cálculo da intensidade máxima do pixel para a linha atual
        for (int y = 0; y < image_size[1]; ++y) {
            PixelType pixelValue = gaussianImage->GetPixel({{xPosition, y}});
            if (pixelValue > maxIntensity) {
                maxIntensity = pixelValue;  
            }
        }

        //std::cout << "Max intesnsity of Line " << i << ": " << maxIntensity << std::endl;

        //Cálculo do threashold para a linha em função da intensidade máxima
        unsigned int threashold = maxIntensity / 2;
        //Iterar na linha, descobrir o número de blocos a considerar, as coordenadas de
        //onde o bloco começa e acaba e calcular a soma de intensidades de cada bloco
        for (int y = 0; y < image_size[1]; ++y) {
        PixelType pixelValue = gaussianImage->GetPixel({{xPosition, y}});
        if (pixelValue > threashold) {
            if (!blockStart) {
                blockNum++;
                blockStart = true;
                blockStartPixel = y;
                //std::cout << "Start pixel of Line " << i << ": " << blockStartPixel << std::endl;
            }
            blockEndPixel = y;
            blockSum += pixelValue;
        } else {
            if (blockStart) {
                blockStart = false;
                blockPixelPairs[i].push_back(std::make_pair(blockStartPixel, blockEndPixel));
                intensitySums[i].push_back(blockSum);
                blockSum = 0;
                //std::cout << "End pixel of Line " << i << ": " << blockEndPixel << std::endl;
            }
        }
    }
        //std::cout << "Num bolcks of Line " << i << ": " << blockNum << std::endl;
    }

    //Descomentar se quiser dar output de das coordenadas de inicio e fim de todos os 
    //os blocos, bem como a intensidade de cada um.
    /*
    for (int i = 0; i < numLines; ++i) {
        std::cout << "Line " << i << " Block Pixel Pairs and Intensity Sums: ";
        for (size_t j = 0; j < blockPixelPairs[i].size(); ++j) {
            std::cout << "(" << blockPixelPairs[i][j].first << ", " << blockPixelPairs[i][j].second << ": "
                      << intensitySums[i][j] << ") ";
        }
        std::cout << std::endl;
    }
    */

    //Para cada linha encontra qual é a soma de intensidades máxima e a que bloco corresponde, e a coordenada média desse bloco
    for (int i = 0; i < numValidLines; ++i) {
        //Index do bloco com soma de intensidade maxima
        int maxIntensityIndex = std::distance(intensitySums[i].begin(), std::max_element(intensitySums[i].begin(), intensitySums[i].end()));
        //Inicio e fim do bloco com soma de intensidade maxima
        std::pair<int, int> highestIntensityPixelPairs = blockPixelPairs[i][maxIntensityIndex];
        // Output
        /*
        std::cout << "Line " << i << ": Highest intensity is " << intensitySums[i][maxIntensityIndex]
                  << " at pixel pair (" << highestIntensityPixelPairs.first << ", " << highestIntensityPixelPairs.second << ")" << std::endl;
        */
        //Cálculo da coordenada média do bloco com a soma de intensidades mais alta
        //midPixel_y[i] = (highestIntensityPixelPairs.first + highestIntensityPixelPairs.second) / 2;
        midPixel_y[i] = highestIntensityPixelPairs.second;

    }

    //Pontos para regressao
    for (int i = 0; i < numValidLines; ++i) {
        unsigned int xPosition = min_coordx + static_cast<int>(i * spacing);
        unsigned int mid_y = midPixel_y[i];
        pointsToFit.push_back(std::make_pair(xPosition, mid_y));
    }

    /*
    for (int i = 0; i < numValidLines; ++i) {
        std::cout << "(" << pointsToFit[i].first << ", " << pointsToFit[i].second << ")" << std::endl;
    }
    */
    return pointsToFit;
}


void WritePointCloudToFile(const std::vector<Eigen::Vector3d> &point_cloud, const std::string &filename)
{
    std::ofstream file;
    file.open(filename);

    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return;
    }

    for (const auto &point : point_cloud)
    {
        file << point(0) << " " << point(1) << " " << point(2) << "\n";
    }

    file.close();

    std::cout << "Point cloud written to " << filename << std::endl;
}

int CreatePointCloud(ProcessingMessage *processor, Eigen::MatrixXd calibration_matrix)
{
    std::vector<Eigen::Vector3d> point_cloud;

    for (const auto &observation : processor->list_of_recorded_points)
    {
        Eigen::Matrix4d pose = observation.pose;
        pose.block<3,1>(0,3) *= 0.001;
        for (const auto &segmented_point : observation.segmented_points)
        {
            // Coordenadas da imagem
            unsigned int x = segmented_point.first;
            unsigned int y = segmented_point.second;
            constexpr double pixel_size = 0.00018867924; // confirmar
            Eigen::Vector4d image_point(pixel_size*x, pixel_size*y, 0, 1.0);
            Eigen::Vector4d point_in_world_with_1 = pose * calibration_matrix * image_point;
            Eigen::Vector3d point_in_world = point_in_world_with_1.head<3>();

            point_cloud.push_back(point_in_world);
        }
    }

    WritePointCloudToFile(point_cloud, "Pointcloud.txt");

    std::cout << "Generated point cloud: " << std::endl;
    for (const auto &point : point_cloud)
    {
        std::cout << "Point: [" << point(0) << ", " << point(1) << ", " << point(2) << "]" << std::endl;
    }

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
    auto time = ts->GetNanosecond();
    // std::cout << time << std::endl;

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

    // Pixel x médio
    ImageType::SizeType size_itk = shr_ptr_imported->GetLargestPossibleRegion().GetSize();
    float cx = (size_itk[0] / 2.0f);

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
    std::vector<std::pair<unsigned int, unsigned int>> segmented_points = segment_points(processor->min_coordx, processor->max_coordx, processor->numLines, shr_ptr_imported);

    // Matriz da flange
    igtl::Matrix4x4 local_mat;
    message_body->GetMatrix(local_mat);

    Eigen::Matrix4d eigen_mat;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            eigen_mat(i, j) = local_mat[i][j];
        }
    }
    //std::cout << "message with transform:\n" << eigen_mat << std::endl; 
    /*
//Output da matriz da flange em tempo real
std::cout << "Observation:" << std::endl;
for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
        std::cout << local_mat[row][col] << " ";
}
std::cout << std::endl;
}
*/

    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - processor->start_time).count();
    processor->timer = elapsed_seconds; // Este timer é usado para a GUI

    ObservationEigenFormat observation_n;
    if (processor->start_calibration)
    {
        observation_n.pose = eigen_mat;
        observation_n.segmented_points = segmented_points;
        processor->list_of_recorded_points.push_back(observation_n);
        std::cout << "Captured frame " << processor->list_of_recorded_points.size() << std::endl;
    }

    if (processor->compute_poincloud && !(processor->pointcloud_finished))
    {

        nlohmann::json calibration_data;
        std::ifstream in(CURAN_COPIED_RESOURCE_PATH "/spatial_calibration.json");

        if (!in.is_open())
        {
            std::cout << "failure to open configuration file\n";
            return 1;
        }

        in >> calibration_data;
        std::string timestamp = calibration_data["timestamp"];
        std::string homogenenous_transformation = calibration_data["homogeneous_transformation"];
        double error = calibration_data["optimization_error"];
        std::printf("Using calibration with average error of : %f\n on the date ", error);
        std::cout << timestamp << std::endl;
        std::stringstream matrix_strm;
        matrix_strm << homogenenous_transformation;
        auto calibration_matrix = curan::utilities::convert_matrix(matrix_strm, ',');
        std::cout << "with the homogeneous matrix :\n"
                  << calibration_matrix << std::endl;

        //Eigen::Matrix4d calibration_matrix = Eigen::Matrix4d::Identity();
        CreatePointCloud(processor, calibration_matrix);
        processor->pointcloud_finished.store(true);
    }

    auto buff = curan::utilities::CaptureBuffer::make_shared(shr_ptr_imported->GetBufferPointer(), shr_ptr_imported->GetPixelContainer()->Size() * sizeof(char), shr_ptr_imported);
    curan::ui::ImageWrapper wrapper{buff, size_itk[0], size_itk[1], SkColorType::kGray_8_SkColorType, SkAlphaType::kPremul_SkAlphaType};
    // curan::ui::ImageWrapper wrapper{buff,size_itk[0],size_itk[1]};
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

void ProcessingMessage::attempt_stop()
{
    io_context.stop();
}
