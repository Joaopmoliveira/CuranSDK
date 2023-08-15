#ifndef TEMPLATEDGMR_HEADER_H
#define TEMPLATEDGMR_HEADER_H

#include "utils/Reader.h"
#include <string>
#include <nlohmann/json.hpp>

namespace curan {
namespace gaussian {

template<int size_in, int size_out>
class GMR {
public:

GMR() : nbStates{ 1 } {
}

~GMR() {

}

constexpr int input_size() const {
	assert(model.is_usable==true);
	return in_size;
}

constexpr int output_size() const {
	assert(model.is_usable==true);
	return out_size;
}

inline int components() const {
	assert(model.is_usable==true);
	return nbStates;
}

constexpr double internal_pi = 3.14159265359;

double gaussianPDF(const Eigen::Matrix<double, size_in, 1>& input,size_t k){
	double normalization = 1.0/std::sqrt(2*internal_pi*nonlinear_activation_detsigma[k])
	auto error = input-muk[k];
	return normalization*std::exp(-0.5*error.transpose()*invSigmak[k]*error);
}

Eigen::Matrix<double, size_out, 1> likeliest(const Eigen::Matrix<double, size_in, 1>& input)
{
	assert(model.is_usable==true);
	std::lock_guard<std::recursive_mutex> guard(mut);
	Eigen::Matrix<double, Eigen::Dynamic, 1> H = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(components(), 1);
				
	// Loop through the gaussians to compute the posterior weights	
	double sum = 0.0;
	for (int k = 0; k < components(); ++k){
		H(k, 0) = model_priors[k] * gaussianPDF(input, k);
		sum += H(k, 0);
	}
	// std::cout << "unscalled H " << H.transpose() << std::endl;
	H /= sum + std::numeric_limits<double>::epsilon();
	//now we have the activation functions so we can compute the linear contributions of each gaussian.
	Eigen::Matrix<double, size_out, Eigen::Dynamic> LinearPrediction = Eigen::Matrix<double, size_out, Eigen::Dynamic>::Zero(get_output_size(), components());
	out_mean = Eigen::Matrix<double, size_out, 1>::Zero();
	for (int i = 0; i < components(); ++i) { //loop through the gaussians to compute the expected value
		LinearPrediction.col(i) = (linear_offset[i] + linear_prediction[i] * input);
		out_mean += LinearPrediction.col(i) * H(i, 0);
	}
    return out_mean;
};

friend std::istream& operator >> (std::istream& in, GMR& model) 
{
    if(model.is_usable)
        throw std::runtime_error("model already initialized"); 
	nlohmann::json model_to_read = nlohmann::json::parse(in);
	int tempnbVar = model_to_read["in_size"].get<int>();
	if(tempnbVar != model.input_size())
		throw std::runtime_error("missmatch between input size and internal model size");
	tempnbVar = model_to_read["out_size"].get<int>();
	if(tempnbVar != model.output_size())
		throw std::runtime_error("missmatch between output size and internal model size");
	model.nbStates = model_to_read["nGaussians"].get<int>();

    for(size_t gauss_ind = 0; gauss_ind < model.nbStates ; ++gauss_ind){
        nlohmann::json state_k = model_to_read["state_"+std::to_string(gauss_ind)];
		std::string Ak = state_k["A"];
		std::stringstream s << Ak;
		model.Ak.push_back(curan::utilities::convert_matrix(s));
		s.str("");
		s.clear();
		std::string bk = state_k["b"];
		s << bk;
		model.bk.push_back(curan::utilities::convert_matrix(s));
		s.str("");
		s.clear();
		std::string Sigmak = state_k["Sigma"];
		s << Sigmak;
		auto Sigma = curan::utilities::convert_matrix(s);
		model.invSigmak.push_back(Sigma.inverse());
		model.nonlinear_activation_detsigma.push_back(Sigma.determinant());
		std::string Muk = state_k["Mu"];
		s.str("");
		s.clear();
		s << Muk;
        model.muk.push_back(curan::utilities::convert_matrix(s));
		double priork = state_k["Prior"];
        model.priork.push_back(priork);
    }
                
	// Model was read successfully.
	model.is_usable = true;
	return in;
};

private:

	std::vector<Eigen::Matrix<double, size_out, size_in>> bk;
	std::vector<Eigen::Matrix<double, size_out, 1>> Ak;
	std::vector<Eigen::Matrix<double, size_in, size_in>> invSigmak;
	std::vector<double> nonlinear_activation_detsigma;
	std::vector<Eigen::Matrix<double, size_in, 1>> muk;
	std::vector<double> priork;

	double regularization_term = 0.0;
	int nbStates = 1;
	static constexpr int in_size = size_in;
	static constexpr int out_size = size_out;

	std::recursive_mutex mut;
};


}
}
#endif