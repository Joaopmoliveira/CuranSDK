#include "gaussianmixtures/GMR.h"
#include <fstream>
#include <iostream>

constexpr size_t in_size = 2;
constexpr size_t out_size = 2;
constexpr bool print_maximum = true;

int main2()
{
    curan::gaussian::GMR<in_size, out_size> model;
    std::ifstream modelfile{CURAN_COPIED_RESOURCE_PATH "/gaussianmixtures_testing/mymodel.txt"};
    modelfile >> model;
    for (size_t i = 0; i < model.components(); ++i)
    {
        std::printf("\n==============\nComponent %llu\n", i);
        std::printf("==============\n-Prior %f\n", model.priork[i]);
        std::printf("\n==============\n-Mu \n==============\n");
        std::cout << model.muk[i];
        std::printf("\n==============\n-Sigma\n==============\n");
        std::cout << model.invSigmak[i].inverse();
        std::printf("\n==============\n-Ak\n==============\n");
        std::cout << model.Ak[i];
        std::printf("\n==============\n-bk\n==============\n");
        std::cout << model.bk[i];
    }

    std::ifstream testfile{CURAN_COPIED_RESOURCE_PATH "/gaussianmixtures_testing/testmymodel.txt"};
    nlohmann::json testing = nlohmann::json::parse(testfile);
    size_t number_of_tests = testing["nTests"];
    double total_error = 0.0;
    constexpr bool print_input_output = false;
    double max_error = -100000.0;
    size_t index = std::numeric_limits<size_t>::max();
    for (size_t it = 0; it < number_of_tests; ++it)
    {
        nlohmann::json test = testing["test" + std::to_string(it + 1)];
        std::stringstream s;
        std::string input = test["input"];
        s << input;
        auto InputMat = curan::utilities::convert_matrix(s);
        std::string output = test["output"];
        s = std::stringstream{};
        s << output;
        auto ExpectedOutputMat = curan::utilities::convert_matrix(s);
        auto ConcreteOutput = model.likeliest(InputMat);
        if (print_input_output)
            std::cout << "\nInput\n"
                      << InputMat << "\nCpp Output\n"
                      << ConcreteOutput << "\nReal Output\n"
                      << ExpectedOutputMat << "\n";
        auto local_error = (ExpectedOutputMat - ConcreteOutput);
        auto squared_local_error = local_error.transpose().eval() * local_error;
        total_error += squared_local_error(0, 0);
        if (max_error < squared_local_error(0, 0))
        {
            index = it;
            max_error = squared_local_error(0, 0);
        }
        if (print_input_output)
            std::cout << "Error: " << local_error << "\n";
    }

    for (const auto &det : model.nonlinear_activation_detsigma)
        std::cout << "det :" << det << "\n";

    std::printf("\nTotal Error : %f\nAverage Error: %f\n Max Error : %f index : %lld\n", total_error, total_error / number_of_tests, max_error, index);
    return 0;
}

int main()
{
    curan::gaussian::GMR<in_size, out_size> model;
    std::ifstream modelfile{CURAN_COPIED_RESOURCE_PATH "/gaussianmixtures_testing/model.txt"};
    modelfile >> model;

    std::ifstream testfile{CURAN_COPIED_RESOURCE_PATH "/gaussianmixtures_testing/testmymodel.txt"};
    nlohmann::json testing = nlohmann::json::parse(testfile);

    {
        size_t number_of_tests = testing["nTests"];
        double total_error = 0.0;
        constexpr bool print_input_output = false;
        double max_error = 0.0;
        size_t index = 0;
        for (size_t it = 0; it < number_of_tests; ++it)
        {
            nlohmann::json test = testing["test" + std::to_string(it + 1)];
            std::stringstream s;
            std::string input = test["input"];
            s << input;
            auto InputMat = curan::utilities::convert_matrix(s);
            std::string output = test["output"];
            s = std::stringstream{};
            s << output;
            auto ExpectedOutputMat = curan::utilities::convert_matrix(s);
            auto ConcreteOutput = model.likeliest_robust(InputMat);
            if (print_input_output)
                std::cout << "\nInput\n"
                          << InputMat << "\nCpp Output\n"
                          << ConcreteOutput << "\nReal Output\n"
                          << ExpectedOutputMat << "\n";
            auto local_error = ExpectedOutputMat - ConcreteOutput;
            auto squared_local_error = local_error.norm();
            if (it == 0)
            {
                std::cout << "\nInput\n"
                          << InputMat << "\nCpp Output\n"
                          << ConcreteOutput << "\nReal Output\n"
                          << ExpectedOutputMat << "\n";
            }

            // std::printf("\nerror mememem%f\n",squared_local_error);
            total_error += squared_local_error;
            if (max_error < squared_local_error)
            {
                index = it;
                max_error = squared_local_error;
            }
            if (print_input_output)
                std::cout << "Error: " << local_error << "\n";
        }

        std::cout << "\nMaximum error validation with robust implementation:\n";
        std::printf("\nTotal Error : %f\nAverage Error: %f\n Max Error : %f\n", total_error, total_error / (double)number_of_tests, max_error);
    }

    {
        size_t number_of_tests = testing["nTests"];
        double total_error = 0.0;
        constexpr bool print_input_output = false;
        double max_error = 0.0;
        size_t index = 0;
        for (size_t it = 0; it < number_of_tests; ++it)
        {
            nlohmann::json test = testing["test" + std::to_string(it + 1)];
            std::stringstream s;
            std::string input = test["input"];
            s << input;
            auto InputMat = curan::utilities::convert_matrix(s);
            std::string output = test["output"];
            s = std::stringstream{};
            s << output;
            auto ExpectedOutputMat = curan::utilities::convert_matrix(s);
            auto ConcreteOutput = model.likeliest(InputMat);
            if (print_input_output)
                std::cout << "\nInput\n"
                          << InputMat << "\nCpp Output\n"
                          << ConcreteOutput << "\nReal Output\n"
                          << ExpectedOutputMat << "\n";
            auto local_error = ExpectedOutputMat - ConcreteOutput;
            auto squared_local_error = local_error.norm();
            if (it == 0)
            {
                std::cout << "\nInput\n"
                          << InputMat << "\nCpp Output\n"
                          << ConcreteOutput << "\nReal Output\n"
                          << ExpectedOutputMat << "\n";
            }

            // std::printf("\nerror mememem%f\n",squared_local_error);
            total_error += squared_local_error;
            if (max_error < squared_local_error)
            {
                index = it;
                max_error = squared_local_error;
            }
            if (print_input_output)
                std::cout << "Error: " << local_error << "\n";
        }

        std::cout << "\nMaximum error validation:\n";
        std::printf("\nTotal Error : %f\nAverage Error: %f\n Max Error : %f\n", total_error, total_error / (double)number_of_tests, max_error);
    }

    return 0;
}