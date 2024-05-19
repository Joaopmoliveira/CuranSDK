#include <iostream>
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include <cmath>
#include <ctime>

//Definir types
constexpr unsigned int Dimension = 2;
using PixelType = unsigned int;
//using PixelType = float;
using ImageType = itk::Image<PixelType, Dimension>;

//Funçao que lê um png (irrelevante na implementacao final)
ImageType::Pointer image_read(){
    //Diretorias 
    const char* filePath = "C:/dev/HumanRoboticsSDK/Curan/tests/temporal_calibration/frame100.png";

    //Definir types
    using ReaderType = itk::ImageFileReader<ImageType>;
    using WriterType = itk::ImageFileWriter<ImageType>;

    //Ler imagem
    ReaderType::Pointer reader = ReaderType::New();
    reader->SetFileName(filePath);
    try
    {
        reader->Update();
    }
    catch (itk::ExceptionObject& e)
    {
        std::cerr << "Error reading the file: " << e << std::endl;
       //return EXIT_FAILURE;
    }

    //Ponteiro para a imagem
    ImageType::Pointer input_image = reader->GetOutput();
    return input_image;
    }

//Funçao que segmenta os pontos 
std::vector<std::pair<unsigned int, unsigned int>> segment_points(int min_coordx, int max_coordx, int numLines, ImageType::Pointer input_image){

    //Dimensões da imagem
    ImageType::SizeType image_size = input_image->GetLargestPossibleRegion().GetSize();
    //std::cout << "Image Dimensions: "
            //  << image_size[0] << " x " << image_size[1] << std::endl;

    //Cálculo do espaçamento entre linhas
    double spacing = static_cast<double>(max_coordx - min_coordx) / (numLines-1);

    // Initialize a vector to store the x positions of valid lines
    std::vector<unsigned int> validXPositions;

    // Calculate the actual number of valid lines
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
        // Save the x position of the valid line
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
            PixelType pixelValue = input_image->GetPixel({{xPosition, y}});
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
        PixelType pixelValue = input_image->GetPixel({{xPosition, y}});
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
        midPixel_y[i] = (highestIntensityPixelPairs.first + highestIntensityPixelPairs.second) / 2;
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

//RANSAC algorithm 
std::vector<double> RANSAC(std::vector<std::pair<unsigned int, unsigned int>> points, int numIterations, double inlierThreshold) {
    //Inicializar variáveis 
    std::vector<double> bestModelParams;
    int bestNumInliers = 0;

    //Numero random em funcao do current time
    std::srand(std::time(0)); 

    for (int iter = 0; iter < numIterations; ++iter) {
        //Dois indexs random de 0 ao size da amostra
        int index1 = std::rand() % points.size();
        int index2 = std::rand() % points.size();

        //Passa a iteração se os indexs por acaso sairem iguais
        if (index1 == index2) continue;

        //Fit dos dois pontos (cálculo dos parametros do modelo)
        Eigen::MatrixXd Xaumentado(2, 2);
        Eigen::VectorXd Y(2);
        Xaumentado << points[index1].first, 1,
        points[index2].first, 1;
        Y << points[index1].second,
             points[index2].second;

        //Metodo "tradicional" 
        //Eigen::MatrixXd Xaumentadopseudoinversa = Xaumentado.completeOrthogonalDecomposition().pseudoInverse();
        //Eigen::VectorXd x = Xaumentadopseudoinversa * Y;
        //Tempo equivalente ao anterior
        //Eigen::VectorXd x = Xaumentado.colPivHouseholderQr().solve(Y); 
        //SVD - não é melhor que o lu em termos de tempo 
        //Eigen::JacobiSVD<Eigen::MatrixXd> svd(Xaumentado, Eigen::ComputeThinU | Eigen::ComputeThinV);
        //Eigen::VectorXd x = svd.solve(Y);

        Eigen::VectorXd x = Xaumentado.lu().solve(Y); //3x mais rapido que calcular pseudoinversa +/-

        // Contar inliers
        int numInliers = 0;
        for (const auto& point : points) {
            double predictedY = x[0] * point.first + x[1];
            double error = std::abs(point.second - predictedY);
            if (error <= inlierThreshold) {
                numInliers++;
            }
        }
        //Melhor modelo é o que tem o maior numero de inliers
        if (numInliers > bestNumInliers) {
            bestModelParams = {x[0], x[1]};
            bestNumInliers = numInliers;
        }
    }
    return bestModelParams;
}

int main(){
    using namespace std::chrono;    

    //Ler imagem (irrelevante na implementação final)
    auto start1 = high_resolution_clock::now();
    ImageType::Pointer input_image = image_read();
    auto stop1 = high_resolution_clock::now();
    auto duration1 = duration_cast<microseconds>(stop1 - start1);
    std::cout << "Tempo ler imagem:" << duration1.count() << std::endl;

    //Segmentaçao de pontos 
    //Janela de scan e número de linhas de avaliação
    int min_coordx = 210;
    int max_coordx = 610;
    int numLines = 40;

    auto start2 = high_resolution_clock::now();
    std::vector<std::pair<unsigned int, unsigned int>> pointsToFit = segment_points(min_coordx, max_coordx, numLines, input_image);
    auto stop2 = high_resolution_clock::now();
    auto duration2 = duration_cast<microseconds>(stop2 - start2);
    std::cout << "Tempo segmentacao:" << duration2.count() << std::endl;

    //RANSAC parameters
    int numIterations = 400;
    double inlierThreshold = 1.0;
    auto start = high_resolution_clock::now();
    std::vector<double> bestModelParams = RANSAC(pointsToFit, numIterations, inlierThreshold);

    //Resultados
    std::cout << "Best model parameters: " << bestModelParams[0] << " " << bestModelParams[1] << std::endl;
    double signal_value = bestModelParams[0] * 410 + bestModelParams[1];
    std::cout << "Signal value: " << signal_value << std::endl;

    return 0;
}