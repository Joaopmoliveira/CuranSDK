#ifndef TEMPLATEDGMR_HEADER_H
#define TEMPLATEDGMR_HEADER_H

#include "utils/Reader.h"
#include <string>
#include <nlohmann/json.hpp>
#include <memory>
#include <mutex>
#include <iostream>

namespace curan
{
	namespace gaussian
	{

		constexpr double internal_pi = 3.14159265359;
		constexpr double epsilon_acceptable = 0.001;

		template <int size_in, int size_out>
		class GMR
		{
		public:
			GMR() : nbStates{1}
			{
			}

			~GMR()
			{
			}

			constexpr int input_size() const
			{
				return in_size;
			}

			constexpr int output_size() const
			{
				return out_size;
			}

			inline int components() const
			{
				assert(is_usable == true);
				return nbStates;
			}

			template <bool debug_print>
			double gaussianPDF(const Eigen::Matrix<double, size_in, 1> &input, size_t k)
			{
				double normalization = std::sqrt(std::pow(2 * internal_pi, size_in) * nonlinear_activation_detsigma[k] + std::numeric_limits<double>::epsilon());
				if (debug_print)
					std::cout << "\nNormalization pi val :" << std::pow(2 * internal_pi, size_in) << "\n";
				if (debug_print)
					std::cout << "\ndeterminant now" << nonlinear_activation_detsigma[k] << "\n";
				if (debug_print)
					std::cout << "\nnormalization\n"
							  << 1.0 / normalization << "\n";
				if (debug_print)
					std::cout << "\ninput \n"
							  << input << "\n mean " << muk[k] << "\n";
				auto error = input - muk[k];
				if (debug_print)
					std::cout << "\nerror \n"
							  << error << "\n";
				if (debug_print)
					std::cout << "inverse \n " << invSigmak[k] << "\n";
				double exponential_argument = -0.5 * error.transpose().eval() * invSigmak[k] * error;
				if (debug_print)
					std::cout << "argument \n"
							  << exponential_argument << std::endl;
				double result = (1.0 / normalization) * std::exp(exponential_argument);
				return result;
			}

			template <bool debug_print>
			double gaussianPDFshifted(const Eigen::Matrix<double, size_in, 1> &input, size_t k,double shift)
			{
				double normalization = std::sqrt(std::pow(2 * internal_pi, size_in) * nonlinear_activation_detsigma[k] + std::numeric_limits<double>::epsilon());
				if (debug_print)
					std::cout << "\nNormalization pi val :" << std::pow(2 * internal_pi, size_in) << "\n";
				if (debug_print)
					std::cout << "\ndeterminant now" << nonlinear_activation_detsigma[k] << "\n";
				if (debug_print)
					std::cout << "\nnormalization\n"
							  << 1.0 / normalization << "\n";
				if (debug_print)
					std::cout << "\ninput \n"
							  << input << "\n mean " << muk[k] << "\n";
				auto error = input - muk[k];
				if (debug_print)
					std::cout << "\nerror \n"
							  << error << "\n";
				if (debug_print)
					std::cout << "inverse \n " << invSigmak[k] << "\n";
				double exponential_argument = -0.5 * error.transpose().eval() * invSigmak[k] * error;
				if (debug_print)
					std::cout << "argument \n"
							  << exponential_argument << std::endl;
				double result = (1.0 / normalization) * std::exp(exponential_argument + shift);
				return result;
			}

			double max_quadraticPDFterm(const Eigen::Matrix<double, size_in, 1> &input)
			{
				double exponential_argument = -std::numeric_limits<double>::max();
				for(size_t index = 0; index < components() ; ++index){
					double quadratic_index = -0.5 * (input - muk[index]).transpose() * invSigmak[index] * (input - muk[index]);
					if(quadratic_index>exponential_argument)
						exponential_argument = quadratic_index;
				}
				return exponential_argument;
			}

			template <bool debug_print, bool robust>
			Eigen::Matrix<double, size_out, 1> likeliest(const Eigen::Matrix<double, size_in, 1> &input)
			{
				assert(is_usable == true);
				Eigen::Matrix<double, Eigen::Dynamic, 1> H = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(components(), 1);

				if constexpr (robust)
				{
					// Loop through the gaussians to compute the posterior weights
					double quadratic_term = -max_quadraticPDFterm(input);
					double sum = 0.0;
					for (int k = 0; k < components(); ++k)
					{
						H(k, 0) = priork[k] * gaussianPDFshifted<debug_print>(input, k,quadratic_term);
						sum += H(k, 0);
					}
					if (debug_print)
					{
						std::cout << H << std::endl;
						std::cout << "\nsum: " << sum << std::endl;
					}

					H /= sum;
				}
				else
				{
					// Loop through the gaussians to compute the posterior weights
					double sum = 0.0;
					for (int k = 0; k < components(); ++k)
					{
						H(k, 0) = priork[k] * gaussianPDF<debug_print>(input, k);
						sum += H(k, 0);
					}
					if (debug_print)
					{
						std::cout << H << std::endl;
						std::cout << "\nsum: " << sum << std::endl;
					}

					H /= sum;
				}

				Eigen::Matrix<double, size_out, 1> out_mean = Eigen::Matrix<double, size_out, 1>::Zero();
				for (int k = 0; k < components(); ++k)
				{
					out_mean += H(k, 0) * (Ak[k] * input + bk[k]);
					if (debug_print)
						std::printf("state %d Likelihood %f\n", k, H(k, 0));
				}

				return out_mean;
			};

			Eigen::Matrix<double, size_out, 1> likeliest(const Eigen::Matrix<double, size_in, 1> &input)
			{
				return likeliest<false,false>(input);
			};

			Eigen::Matrix<double, size_out, 1> likeliest_robust(const Eigen::Matrix<double, size_in, 1> &input)
			{
				return likeliest<false, true>(input);
			};

			friend std::istream &operator>>(std::istream &in, GMR &model)
			{
				if (model.is_usable)
					throw std::runtime_error("model already initialized");
				nlohmann::json model_to_read = nlohmann::json::parse(in);
				int tempnbVar = model_to_read["insize"];
				if (tempnbVar != model.input_size())
					throw std::runtime_error("missmatch between input size and internal model size");
				tempnbVar = model_to_read["outsize"];
				if (tempnbVar != model.output_size())
					throw std::runtime_error("missmatch between output size and internal model size");
				model.nbStates = model_to_read["nGaussians"].get<int>();

				for (size_t gauss_ind = 0; gauss_ind < model.nbStates; ++gauss_ind)
				{
					nlohmann::json state_k = model_to_read["state_" + std::to_string(gauss_ind + 1)];
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
				// check that the priors sum up to one
				double sum = 0.0;
				for (const auto &val : model.priork)
					sum += val;
				if (sum < 1.0 - epsilon_acceptable || sum > 1.0 + epsilon_acceptable)
					throw std::runtime_error("The priors from the file dont sum up to one (sum of priors) : " + std::to_string(sum));
				// Model was read successfully.
				model.is_usable = true;
				return in;
			};

			friend std::ostream& operator<<(std::ostream& os, GMR const & model){
				 os << "usable: " << model.is_usable << std::endl;
				 os << "==========" << std::endl;
				 for(size_t i = 0; i < model.components(); ++i){
					os << ">> prior: " << model.priork[i] << std::endl;
					os << ">> mu: " << model.muk[i].transpose() << std::endl;
					os << ">> sigma: \n" << model.invSigmak[i].inverse() << std::endl;
					os << ">> A: \n" << model.Ak[i] << std::endl;
					os << ">> b: " << model.bk[i].transpose() << std::endl;
					os << "==========" << std::endl;
				 }
        		return os;
    		}


		public:
			volatile bool is_usable = false;

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
		};

	}
}
#endif