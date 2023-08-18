#ifndef TEMPLATEDGMR_HEADER_H
#define TEMPLATEDGMR_HEADER_H

#include "utils/Reader.h"
#include <string>
#include <nlohmann/json.hpp>
#include <memory>
#include <mutex>

namespace curan {
namespace gaussian {


constexpr double internal_pi = 3.14159265359;
constexpr double epsilon_acceptable = 0.001;

template<int size_in, int size_out>
class GMR {
public:

GMR() : nbStates{ 1 } {
}

~GMR() {

}

constexpr int input_size() const {
	return in_size;
}

constexpr int output_size() const {
	return out_size;
}

inline int components() const {
	assert(is_usable==true);
	return nbStates;
}

double gaussianPDF(const Eigen::Matrix<double, size_in, 1>& input,size_t k){
	double normalization = 1.0/std::sqrt(std::pow(2.0*internal_pi,output_size()+input_size())*nonlinear_activation_detsigma[k]);
	auto error = input-muk[k];
	return normalization*std::exp(-0.5*error.transpose()*invSigmak[k]*error);
}

Eigen::Matrix<double, size_out, 1> likeliest(const Eigen::Matrix<double, size_in, 1>& input)
{
	std::lock_guard<std::recursive_mutex> guard(mut);
	assert(is_usable==true);
	Eigen::Matrix<double, Eigen::Dynamic, 1> H = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(components(), 1);
				
	// Loop through the gaussians to compute the posterior weights	
	double sum = 0.0;
	for (int k = 0; k < components(); ++k){
		H(k, 0) = priork[k] * gaussianPDF(input, k);
		sum += H(k, 0);
	}

	H /= sum + std::numeric_limits<double>::epsilon();

	Eigen::Matrix<double, size_out, 1> out_mean = Eigen::Matrix<double, size_out, 1>::Zero();
	for (int k = 0; k < components(); ++k)
		out_mean +=  H(k, 0)*(Ak[k]*input+bk[k]);
	
    return out_mean;
};

friend std::istream& operator >> (std::istream& in, GMR& model) 
{
    if(model.is_usable)
        throw std::runtime_error("model already initialized"); 
	nlohmann::json model_to_read = nlohmann::json::parse(in);
	int tempnbVar = model_to_read["insize"];
	if(tempnbVar != model.input_size())
		throw std::runtime_error("missmatch between input size and internal model size");
	tempnbVar = model_to_read["outsize"];
	if(tempnbVar != model.output_size())
		throw std::runtime_error("missmatch between output size and internal model size");
	model.nbStates = model_to_read["nGaussians"].get<int>();

    for(size_t gauss_ind = 0; gauss_ind < model.nbStates ; ++gauss_ind){
        nlohmann::json state_k = model_to_read["state_"+std::to_string(gauss_ind+1)];
		std::string Ak = state_k["A"];
		std::stringstream s;
		s << Ak;
		auto A = curan::utilities::convert_matrix(s);
		model.Ak.push_back(A);
		s = std::stringstream{};
		std::string bk = state_k["b"];
		s << bk;
		auto b = curan::utilities::convert_matrix(s);
		model.bk.push_back(b);
		s = std::stringstream{};
		std::string Sigmak = state_k["Sigma"];
		s << Sigmak;
		auto Sigma = curan::utilities::convert_matrix(s);
		model.invSigmak.push_back(Sigma.inverse());
		model.nonlinear_activation_detsigma.push_back(Sigma.determinant());
		std::string Muk = state_k["Mu"];
		s = std::stringstream{};
		s << Muk;
		auto mu = curan::utilities::convert_matrix(s);
        model.muk.push_back(mu);
		std::string priork = state_k["Prior"];
        model.priork.push_back(stod(priork));
    }
	//check that the priors sum up to one
	double sum = 0.0;
	for(const auto & val : model.priork)
		sum+=val;
	if(sum<1.0-epsilon_acceptable ||  sum>1.0+epsilon_acceptable)
		throw std::runtime_error("The priors from the file dont sum up to one (sum of priors) : "+std::to_string(sum));
	// Model was read successfully.
	model.is_usable = true;
	return in;
};

public:

	bool is_usable = false;

	std::vector<Eigen::Matrix<double, size_out, 1>> bk;
	std::vector<Eigen::Matrix<double, size_out, size_in>> Ak;
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