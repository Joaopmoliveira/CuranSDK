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
RANSAC_Output RANSAC(std::vector<std::pair<unsigned int, unsigned int>> points, int numIterations, double inlierThreshold) {
    //Inicializar variáveis 
    //std::vector<double> bestModelParams;
    int bestNumInliers = 0;
    //std::vector<std::pair<unsigned int, unsigned int>> inlierPoints;
    RANSAC_Output output;

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
                //output.inlierPoints.push_back(point);
            }
        }
        //std::cout << numInliers << std::endl;
        //Melhor modelo é o que tem o maior numero de inliers
        if (numInliers > bestNumInliers) {
            output.bestModelParams = {x[0], x[1]};
            bestNumInliers = numInliers;
            output.inlierPoints.clear();
            for (const auto& point : points) {
                double predictedY = x[0] * point.first + x[1];
                double error = std::abs(point.second - predictedY);
                if (error <= inlierThreshold) {
                    output.inlierPoints.push_back(point);
                }
            }
        }
    }
    //std::cout << bestNumInliers << std::endl;
    //std::cout << "Best model parameters: " << bestModelParams[0] << " " << bestModelParams[1] << std::endl;
    return output;
}


//Projeta os vetores posição da flange na direção principal do movimento
int ProjectFlange(ProcessingMessage* processor){
	using namespace Eigen;
	using namespace std;
    //Imprimir flange data e video signal originais (pré data process)
    cout << endl;
    cout << "---------------------------------------" << endl;
    cout << "Flange Data (x,y,z):" << endl;
	for (const auto& observation : processor->list_of_recorded_points) {
        for (size_t lines = 0; lines < 3; ++lines) {
            cout << observation.flange_data(lines, 0) << " ";
        }
        cout << endl;
	}
    
    //Extrair vetores posição da lista de observações
    vector<Vector3d> position_vectors;
    for (const auto& observation : processor->list_of_recorded_points) {
        position_vectors.push_back(observation.flange_data);
    }

    //Daqui para a frente de acordo com a wikipedia e comfirmado que o matlab dá igual :-)
    //Vetor posição médio
    Vector3d mean = Vector3d::Zero();
    for (const auto& vec : position_vectors) {
        mean += vec;
    }
    mean /= position_vectors.size();

    //Centrar os dados tendo em conta a posição média
    MatrixXd centered_data(3, position_vectors.size());
    for (size_t i = 0; i < position_vectors.size(); ++i) {
        centered_data.col(i) = position_vectors[i] - mean;
    }

    //Covariance matrix
    MatrixXd covariance = (centered_data * centered_data.transpose()) / (position_vectors.size() - 1);
    //std::cout << "Covariance Matrix:\n" << covariance<< std::endl

    //Eigendecomposition da covariance matrix
    SelfAdjointEigenSolver<MatrixXd> eigensolver(covariance);

    //Eigenvectors e eigenvalues
    Vector3d eigenvalues = eigensolver.eigenvalues();
    Matrix3d eigenvectors = eigensolver.eigenvectors();

    //Pares de valores prórpios com o respetivo vetor próprio
    vector<pair<double, Vector3d>> eigen_pairs;
    for (int i = 0; i < 3; ++i) {
        eigen_pairs.push_back(make_pair(eigenvalues(i), eigenvectors.col(i)));
    }
   
    //Lambda function para comparar os valores prórpios dos pares
    auto compare = [](const pair<double, Vector3d>& a, const pair<double, Vector3d>& b) {
        return a.first > b.first;
    };

    //Organiza os pares por ordem decrescente de valor próprio 
    sort(eigen_pairs.begin(), eigen_pairs.end(), compare);

    //Printa os pares eigenvalue-eigenvector
    cout << endl;
    cout << "---------------------------------------" << endl;
    cout << "Sorted Eigenvalue-Eigenvector Pairs:" << endl;
    for (const auto& pair : eigen_pairs) {
        cout << "Eigenvalue: " << pair.first << ", Eigenvector: \n" << pair.second << endl;
    }
    cout << endl;
    cout << "---------------------------------------" << endl;
    cout << "Principal Component: " << eigen_pairs[0].second.transpose() << endl;

    //Componente principal é o eigenvector que tem o maior eigen value 
    Vector3d principal_component = eigen_pairs[0].second;

    //Projeta os vetores posicao da flange na direção principal
    for (const auto& vec : position_vectors) {
        /*Uso abs para garantir que o sinal é sempre positivo, no matlab não precisei disso. Por alguma razão
        aqui há casos em que os eigen vectors são simétricos (sentido inverso) do que os que dão no maltab.*/
        double projection = abs(vec.dot(principal_component));
        processor->projections.push_back(projection);
    }

    //Output das projeções
    cout << endl;
    cout << "---------------------------------------" << endl;
    cout << "Projections onto the principal component:" << endl;
    for (const auto& projection : processor->projections) {
        cout << projection << endl;
    }
    return 0;
}

//Normaliza o sinal de vídeo e o sinal de posição (já depois das projeções)
int NormalizeData(ProcessingMessage* processor) {
    //Lambda function para calcular a média 
    auto mean = [](auto data) {
        double sum = 0.0;
        for (double value : data) {
            sum += value;
        }
        return sum / data.size();
    };

    //Lambda function para calcular o desvio padrão
    auto sampleStdDev = [](auto data,double mean) {
        double sumSquaredDiff = 0.0;
        for (double value : data) {
            sumSquaredDiff += pow(value - mean, 2);
        }
        return sqrt(sumSquaredDiff / (data.size() - 1));
    };

    //Normaliza as projeções da flange
    auto mean1 = mean(processor->projections);
    auto sampleStdDev1 = sampleStdDev(processor->projections, mean1);
    for (double value : processor->projections) {
        processor->normalized_position_signal.push_back((value - mean1) / sampleStdDev1);
    }

    //Extrair o sinal de vídeo da lista de observações
    std::vector<double> video_signals;
    for (const auto& observation : processor->list_of_recorded_points) {
        video_signals.push_back(observation.video_signal);
    }

    //Output do sinal de vídeo
    std::cout << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "Video signal:" << std::endl;
    for (const auto& signal : video_signals) {
        std::cout << signal << std::endl;
    }   

    //Normaliza o sinal de vídeo
    auto mean2 = mean(video_signals);
    auto sampleStdDev2 = sampleStdDev(video_signals, mean2);
    for (double value : video_signals) {
        processor->normalized_video_signal.push_back(-((value - mean2) / sampleStdDev2));
    }

    //Output dos sinais normalizados
    std::cout << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "Normalized position signal:"<<std::endl;
    for (double value : processor->normalized_position_signal) {
        std::cout << value << " ";
    }
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "Normalized video signal:"<<std::endl;
    for (double value : processor->normalized_video_signal) {
        std::cout << value << " ";
    }
    return 0;
}

int AlignSignals(ProcessingMessage* processor){
    //Lambda para calcular SSD 
    auto calculateSSD = [](std::vector<double>& signal1, std::vector<double> signal2) {
        double ssd = 0.0;
        for (size_t i = 0; i < signal1.size(); ++i) {
            ssd += std::pow(signal1[i] - signal2[i], 2);
        }
        return ssd;
    };

   //Lambda para shiftar um sinal em n samples
    auto shiftSignal = [](std::vector<double>& signal, int shift) {
        std::vector<double> shifted_signal(signal.size());
        int size = static_cast<int>(signal.size());
        for (size_t i = 0; i < signal.size(); ++i) {
            int shifted_index = (static_cast<int>(i) + shift) % size;
            if (shifted_index < 0)
                shifted_index += size; 
            shifted_signal[i] = signal[shifted_index];
        }
        return shifted_signal;
    };

    //Lambda para dar resample para resolução de 1ms usando interpolação linear
    auto resampleSignal = [&processor](const std::vector<double>& originalSignal) {
        std::vector<double> resampledSignal;
        double timeStepOriginal = 1/processor->fps;
        double timeStepResampled = 0.001; 
        int numSamplesOriginal = static_cast<int>(originalSignal.size());
        int numSamplesResampled = static_cast<int>(numSamplesOriginal * timeStepOriginal / timeStepResampled);

        for (int i = 0; i < numSamplesResampled; ++i) {
            double t = i * timeStepResampled;
            int indexLow = static_cast<int>(t / timeStepOriginal);
            int indexHigh = std::min(indexLow + 1, numSamplesOriginal - 1);
            double fraction = t / timeStepOriginal - indexLow;
            double interpolatedValue = originalSignal[indexLow] * (1 - fraction) + originalSignal[indexHigh] * fraction;
            resampledSignal.push_back(interpolatedValue);
        }
        return resampledSignal;
    };

    //Alinhamento inicial (sample a sample)
    int coarse_resolution = 1; //Saltar de sample em sample
    int coarse_shift_range = processor->normalized_position_signal.size(); 
    double min_coarse_ssd = std::numeric_limits<double>::max(); //iniciar no infinito
    int best_coarse_shift = 0;
    //int iter = 0;

    for (int shift = -coarse_shift_range; shift <= coarse_shift_range; shift += coarse_resolution) {
        //iter+=1;
        std::vector<double> shifted_tracker_position = shiftSignal(processor->normalized_position_signal, shift);
        double ssd = calculateSSD(shifted_tracker_position, processor->normalized_video_signal);
        //std::printf("SSD iteration %lld: ", iter);
        //std::cout << ssd << std::endl;
        if (ssd < min_coarse_ssd) {
            min_coarse_ssd = ssd;
            best_coarse_shift = shift;
            //std::cout << "Best shift iteration: " << best_coarse_shift << std::endl;
            //Um valor negativo significa que o moving signal tá adiantado em relacao ao fixed 
        }
    }

    //Criar o vetor com o sinal de posição corrigido com o alinhamento inicial
    std::vector<double> shifted_signal = shiftSignal(processor->normalized_position_signal, best_coarse_shift);   
    //Dar resample para resolução de 1ms   
    auto resampled_tracker_signal = resampleSignal(shifted_signal); //Já com o alinhamento inicial
    auto resampled_image_signal = resampleSignal(processor->normalized_video_signal);

    //Alinhamento final (com o sinal resampled)
    int fine_resolution = 1; //Saltar de sample em sample
    int fine_shift_range = (1.0/processor->fps)*1000; //shift range igual ao time step porque o alinhamento inicial ja foi feito
    double min_fine_ssd = std::numeric_limits<double>::max(); //iniciar no infinito
    int best_fine_shift = 0;
    //int iter2 = 0;

    for (int shift = -fine_shift_range; shift <= fine_shift_range; shift += fine_resolution) {
        //iter2+=1;
        std::vector<double> shifted_tracker_resampled_position = shiftSignal(resampled_tracker_signal, shift);
        double ssd = calculateSSD(shifted_tracker_resampled_position, resampled_image_signal);
        //std::printf("SSD iteration %lld: ", iter2);
        //std::cout << ssd << std::endl;
        if (ssd < min_fine_ssd) {
            min_fine_ssd = ssd;
            best_fine_shift = shift;
            //std::cout << "Best shift iteration: " << best_fine_shift << std::endl;
            //Um valor negativo significa que o moving signal tá adiantado em relacao ao fixed 
        }
    }
    
    float coarse_alignemnt = best_coarse_shift * (1.0/processor->fps)*1000;
    float fine_alignement = best_fine_shift;
    float total_shift = coarse_alignemnt + fine_alignement;
    std::cout << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "Coarse Shift: " << coarse_alignemnt << std::endl;
    std::cout << "Fine Shift: " << fine_alignement << std::endl;
    std::cout << "Total Shift: " << total_shift << std::endl;
    processor->calibration_value = total_shift;

    return 0;
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
    igtl::TimeStamp::Pointer ts;
    ts = igtl::TimeStamp::New();
    message_body->GetTimeStamp(ts);
    auto time = ts->GetNanosecond();
    //std::cout << time << std::endl;

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

	//Frame
    ImageType::Pointer shr_ptr_imported = importFilter->GetOutput();

    //Segmentation
    std::vector<std::pair<unsigned int, unsigned int>> pointsToFit = segment_points(processor->min_coordx, processor->max_coordx, processor->numLines, shr_ptr_imported);
    
    //RANSAC
    auto RANSAC_output = RANSAC(pointsToFit, processor->numIterations, processor->inlierThreshold);
    auto bestModelParams = RANSAC_output.bestModelParams;
    auto inliers = RANSAC_output.inlierPoints;

    //Pixel x médio 
    ImageType::SizeType size_itk = shr_ptr_imported->GetLargestPossibleRegion().GetSize();
    float cx = (size_itk[0]/2); 
    //Sinal de vídeo atual tendo em conta o fit do RANSAC
    float cy = bestModelParams[0] * cx + bestModelParams[1];

    //Matriz da flange
	igtl::Matrix4x4 local_mat;
	message_body->GetMatrix(local_mat);

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
    
    ObservationEigenFormat observation_n; //Struct com dois parametros: vetor posição da flange e sinal do vídeo
    
    /*Aqui está a lógica de guardar os dados para a calibração e de a executar. Guarda os vetores posição da flange e o sinal de vídeo numa lista em que cada elemento é a struct defenida
    na linha anterior. Isto acontece durante um tempo definido pelo utilizador e tem um delay inicial de 3s.
    Quando acaba de guardar os dados faz os cálculos da calibração.*/
    if (processor->start_calibration && (processor->timer < processor->aquisition_time +3)) {
        processor->calibration_finished.store(false);
        if (processor->timer >= 3){
            std::cout << "Timer: " <<processor->timer<< "s" <<std::endl;
                for (size_t lines = 0; lines < 3; ++lines) {
                    size_t cols = 3;
                    observation_n.flange_data(lines, 0) = local_mat[lines][cols];
                }
            observation_n.video_signal = cy;
            processor->list_of_recorded_points.push_back(observation_n);
            std::cout << "Observation "<< processor->list_of_recorded_points.size() << std::endl;
        }
        double aux = 1;
        processor->timer += aux/processor->fps;
    }else if (processor->start_calibration && !(processor->calibration_finished)) {
        try {
            ProjectFlange(processor);
        } catch (...) {
            std::cerr << "An error occurred during ProjectFlange" << std::endl;
        }

        try {
            NormalizeData(processor);
        } catch (...) {
            std::cerr << "An error occurred during NormalizeData" << std::endl;
        }

        try {
            AlignSignals(processor);
        } catch (...) {
            std::cerr << "An error occurred during AlignSignals" << std::endl;
        }

        std::cout << "---------------------------------------" << std::endl;
        processor->calibration_finished.store(true);
    }

    //std::cout << size_itk << std::endl;

    auto buff = curan::utilities::CaptureBuffer::make_shared(shr_ptr_imported->GetBufferPointer(),shr_ptr_imported->GetPixelContainer()->Size()*sizeof(char),shr_ptr_imported);
    curan::ui::ImageWrapper wrapper{buff,size_itk[0],size_itk[1]};
    processor->processed_viwer->update_batch([inliers, pointsToFit, size_itk, processor,bestModelParams,cx , cy](SkCanvas* canvas, SkRect image_area,SkRect widget_area){ 
        if (processor->show_line){ //Vizualizador da linha
            //Fator de escala
            float scalling_factor_x = image_area.width()/size_itk[0];
            float scalling_factor_y = image_area.height()/size_itk[1];

            //Cálculo dos pontos no referencial da imagem e no referencial do canvas
            float y1 = bestModelParams[0] * processor->min_coordx + bestModelParams[1]; 
            float y2 = bestModelParams[0] * processor->max_coordx + bestModelParams[1]; 
            float x1canvas = processor->min_coordx * scalling_factor_x + image_area.left();
            float x2canvas = processor->max_coordx * scalling_factor_x + image_area.left();
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
            }

        if (processor->show_calibration_lines){ //GUI que mostra as linhas de scan
            SkPaint linePaint;
            linePaint.setStyle(SkPaint::kStroke_Style);
            linePaint.setStrokeWidth(1.0f);  
            SkColor customColor = SkColorSetARGB(255, 255, 128, 0);
            linePaint.setColor(customColor); 
            double spacing = static_cast<double>(processor->max_coordx - processor->min_coordx) / (processor->numLines-1);
            std::vector<unsigned int> XPositions;
            for (int i = 0; i < processor->numLines; ++i) {
                unsigned int xPosition = processor->min_coordx + static_cast<int>(i * spacing);
                XPositions.push_back(xPosition);
            }
            float scalling_factor_x = image_area.width()/size_itk[0];
            for (int i = 0; i < processor->numLines ; ++i) {
                canvas->drawLine(XPositions[i]* scalling_factor_x + image_area.left(), 80, XPositions[i]* scalling_factor_x + image_area.left(), widget_area.bottom(), linePaint);
            }
        }

        if(processor->show_pointstofit){ //GUI que mostra os pontos segmentados
            float scalling_factor_x = image_area.width()/size_itk[0];
            float scalling_factor_y = image_area.height()/size_itk[1];
            //Todos os pontos
            SkPaint paint;
            SkColor customColor = SkColorSetARGB(255, 255, 153, 153);
            paint.setColor(customColor); 
            paint.setStyle(SkPaint::kFill_Style); 
            SkScalar radius = 3;
            for (auto& points : pointsToFit){
                canvas->drawCircle(points.first * scalling_factor_x + image_area.left(), points.second * scalling_factor_y + image_area.top(), radius, paint);
            }
            //Inliers pintam por cima de todos os pontos para distinguir os inliers dos outliers
            SkColor customColor2 = SkColorSetARGB(255, 153, 255, 153);
            paint.setColor(customColor2);
            for (auto& inlier : inliers){
                canvas->drawCircle(inlier.first * scalling_factor_x + image_area.left(), inlier.second * scalling_factor_y + image_area.top(), radius, paint);
            }
        }

        if (processor->start_calibration && processor->timer < 3){ //Countdown para a calibração
            SkPaint paint;
            paint.setStyle(SkPaint::kFill_Style);
            SkColor customColor = SkColorSetARGB(255, 178, 102, 255);
            paint.setColor(customColor); 
            const std::string text1 = "Calibration starting in";
            float countdown = 3 - processor->timer;
            SkString text2;
            text2.printf("%.1f", countdown);
            text2 += "s";
            const char* fontFamily = nullptr;  
            SkFontStyle fontStyle;  
            sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
            sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);
            SkFont font1(typeface, 40.0f, 1.0f, 0.0f);
            font1.setEdging(SkFont::Edging::kAntiAlias);
            canvas->drawSimpleText(text1.data(),text1.size(),SkTextEncoding::kUTF8,widget_area.centerX()-200,widget_area.top()+50,font1,paint);
            canvas->drawSimpleText(text2.data(),text2.size(),SkTextEncoding::kUTF8,widget_area.centerX()-40,widget_area.top() + 90,font1,paint);
        
        }else if (processor->start_calibration && (processor->timer > 3 && processor->timer < processor->aquisition_time + 3)){ //GUI enquanto a calibração tá a acontecer
            SkPaint paint;
            paint.setColor(SK_ColorRED); 
            paint.setStyle(SkPaint::kFill_Style);
            const char* fontFamily = nullptr; 
            SkFontStyle fontStyle;  
            sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
            sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);
            SkFont font1(typeface, 64.0f, 1.0f, 0.0f);
            font1.setEdging(SkFont::Edging::kAntiAlias);
            const std::string text = "REC";
            float percentagem = ((processor->timer -3) / processor->aquisition_time)*100;
            SkString text2;
            text2.printf("%.0f", percentagem);
            text2 += "%";
            SkColor customColor = SkColorSetARGB(255, 176, 196, 222);
            SkPaint paint2;
            paint2.setColor(customColor); 
            paint2.setStyle(SkPaint::kFill_Style);
            canvas->drawCircle(widget_area.left()+190, widget_area.top() + 100, 25, paint);
            canvas->drawSimpleText(text.data(),text.size(),SkTextEncoding::kUTF8,widget_area.left()+30,widget_area.top() + 120,font1,paint);
            canvas->drawSimpleText(text2.data(),text2.size(),SkTextEncoding::kUTF8,widget_area.left()+30,widget_area.top() + 180,font1,paint2);
        
        }else if(processor->calibration_finished){ //GUI quando a calibração acabar
            SkPaint paint;
            paint.setStyle(SkPaint::kFill_Style);
            SkColor customColor = SkColorSetARGB(255, 178, 102, 255);
            paint.setColor(customColor); 
            const char* fontFamily = nullptr; 
            SkFontStyle fontStyle;  
            sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
            sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);
            SkFont font1(typeface, 64.0f, 1.0f, 0.0f);
            font1.setEdging(SkFont::Edging::kAntiAlias);
            const std::string text1 = "Calibration completed!";
            SkString text2;
            float calib = processor->calibration_value;
            text2.printf("Result: %.1f", calib);
            text2 += "ms";
            canvas->drawSimpleText(text1.data(),text1.size(),SkTextEncoding::kUTF8,widget_area.centerX()-300,widget_area.top()+100,font1,paint);
            canvas->drawSimpleText(text2.data(),text2.size(),SkTextEncoding::kUTF8,widget_area.centerX()-150,widget_area.top() + 200,font1,paint);
        }
    },wrapper);

	end = std::chrono::steady_clock::now();
	auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
	if(time_elapsed>40)
		std::printf("Warning: Processing time higher than frame to frame interval (%lld milliseconds)\n",time_elapsed);
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
	auto client = Client::make(construction);
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
	client->connect(lam);
	io_context.run();
	button->set_waiting_color(SK_ColorRED);
	return;
}

void ProcessingMessage::attempt_stop() {
	io_context.stop();
}
