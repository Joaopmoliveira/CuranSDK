#include "MessageProcessing.h"
#include "optimization/WireCalibration.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <ctime>

bool process_transform_message(ProcessingMessage* processor,igtl::MessageBase::Pointer val){
	igtl::TransformMessage::Pointer transform_message = igtl::TransformMessage::New();
	transform_message->Copy(val);
	int c = transform_message->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return false; //failed to unpack message, therefore returning without doing anything
	return true;
}

using PixelType = unsigned char;
constexpr unsigned int Dimension = 2;
using ImageType = itk::Image<PixelType, Dimension>;
//Funçao que segmenta os pontos 
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

bool process_image_message(ProcessingMessage* processor,igtl::MessageBase::Pointer val){
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point end = begin;
	igtl::ImageMessage::Pointer message_body = igtl::ImageMessage::New();
	message_body->Copy(val);
	int c = message_body->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return false; //failed to unpack message, therefore returning without doing anything

	int x, y, z;
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
	const itk::SpacePrecisionType origin[Dimension] = { 0.0, 0.0 };
	importFilter->SetOrigin(origin);
	const itk::SpacePrecisionType spacing[Dimension] = { 1.0, 1.0 };
	importFilter->SetSpacing(spacing);
	
	const bool importImageFilterWillOwnTheBuffer = false;
	importFilter->SetImportPointer((PixelType*)message_body->GetScalarPointer(), message_body->GetScalarSize(), importImageFilterWillOwnTheBuffer);

	try {
		importFilter->Update();
	}
	catch (...) {
		return false;
	}

	// where you have imported the raw image buffer into a ITK compatible image, thus you can use this for your purpouses

	ObservationEigenFormat observation_n;

	igtl::Matrix4x4 local_mat;
	message_body->GetMatrix(local_mat);
	for (size_t cols = 0; cols < 4; ++cols)
		for (size_t lines = 0; lines < 4; ++lines)
			observation_n.flange_data(lines, cols) = local_mat[lines][cols];
		
    /*
	std::cout << "Observation:" << std::endl;
	for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            std::cout << observation_n.flange_data(row, col) << " ";
    }
    std::cout << std::endl;
    }
    */
   
    ImageType::Pointer shr_ptr_imported = importFilter->GetOutput();
    //Segmentation parameters
    int min_coordx = 210;
    int max_coordx = 610;
    int numLines = 20;
    std::vector<std::pair<unsigned int, unsigned int>> pointsToFit = segment_points(min_coordx, max_coordx, numLines, shr_ptr_imported);
    
    //RANSAC parameters
    int numIterations = 400;
    double inlierThreshold = 1.0;
    std::vector<double> bestModelParams = RANSAC(pointsToFit, numIterations, inlierThreshold);
    //std::cout << "Best model parameters: " << bestModelParams[0] << " " << bestModelParams[1] << std::endl;

    ImageType::SizeType size_itk = shr_ptr_imported->GetLargestPossibleRegion().GetSize();
    auto buff = curan::utilities::CaptureBuffer::make_shared(shr_ptr_imported->GetBufferPointer(),shr_ptr_imported->GetPixelContainer()->Size()*sizeof(char),shr_ptr_imported);
    curan::ui::ImageWrapper wrapper{buff,size_itk[0],size_itk[1]};
    processor->processed_viwer->update_batch([size_itk, processor,bestModelParams, min_coordx, max_coordx](SkCanvas* canvas, SkRect image_area,SkRect widget_area){
    if (processor->show_line){
        //Fator de escala
        float scalling_factor_x = image_area.width()/size_itk[0];
        float scalling_factor_y = image_area.height()/size_itk[1];

        //Cálculo dos pontos no referencial da imagem e no referencial do canvas
        float y1 = bestModelParams[0] * min_coordx + bestModelParams[1]; // Calculate y-coordinate for minX
        float y2 = bestModelParams[0] * max_coordx + bestModelParams[1]; // Calculate y-coordinate for maxX
        float cx = (size_itk[0]/2); 
        float cy = bestModelParams[0] * cx + bestModelParams[1];
        float x1canvas = min_coordx * scalling_factor_x + image_area.left();
        float x2canvas = max_coordx * scalling_factor_x + image_area.left();
        float y1canvas = y1 * scalling_factor_y + image_area.top();
        float y2canvas = y2 * scalling_factor_y + image_area.top();
        float cxcanvas = cx * scalling_factor_x + image_area.left();
        float cycanvas = cy * scalling_factor_y + image_area.top();

        //Linha 
        SkPaint linePaint;
        linePaint.setStyle(SkPaint::kStroke_Style);
        linePaint.setAntiAlias(true);
        linePaint.setStrokeWidth(2.0f);  
        linePaint.setColor(SK_ColorBLUE); 
        canvas->drawLine(x1canvas, y1canvas, x2canvas, y2canvas, linePaint);

        //Circulo
        SkPaint paint;
        paint.setColor(SK_ColorRED); 
        paint.setStyle(SkPaint::kFill_Style); 
        SkScalar radius = 3;
        canvas->drawCircle(cxcanvas, cycanvas, radius, paint);
    }else{
        auto nada = 1;
    }
	},wrapper);

	end = std::chrono::steady_clock::now();
	auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
	if(time_elapsed>40)
		std::printf("warning: reduce brightness of image because processing size is too large (%lld milliseconds)\n",time_elapsed);
	return true;
}

std::map<std::string,std::function<bool(ProcessingMessage*,igtl::MessageBase::Pointer)>> openigtlink_callbacks{
	{"TRANSFORM",process_transform_message},
	{"IMAGE",process_image_message}
};

bool ProcessingMessage::process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
	assert(val.IsNotNull());
	if (er){
        return true;
    } 
    if (auto search = openigtlink_callbacks.find(val->GetMessageType()); search != openigtlink_callbacks.end())
        search->second(this,val);
    else
        std::cout << "No functionality for function received\n";
    return false;
}

void ProcessingMessage::communicate() {
	list_of_recorded_points.clear();
	using namespace curan::communication;
	button->set_waiting_color(SK_ColorGREEN);
	io_context.reset();
	interface_igtl igtlink_interface;
	Client::Info construction{ io_context,igtlink_interface };
	asio::ip::tcp::resolver resolver(io_context);
	auto endpoints = resolver.resolve("localhost", std::to_string(port));
	construction.endpoints = endpoints;
	Client client{ construction };
	connection_status.set(true);

	auto lam = [this](size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
		try{
			if (process_message(protocol_defined_val, er, val))
			{
				connection_status.set(false);
				attempt_stop();
			}
		}catch(...){
			std::cout << "Exception was thrown\n";
		}
	};
	client.connect(lam);
	io_context.run();
	button->set_waiting_color(SK_ColorRED);
	return;
}

void ProcessingMessage::attempt_stop() {
	io_context.stop();
}
