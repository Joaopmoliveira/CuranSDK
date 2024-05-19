#include <array>
#include <vector>

enum class Preference{
    VERTICAL,
    MOSTLY_VERTICAL,
    HORIZONTAL,
    MOSTLY_HORIZONTAL,
};

std::vector<std::array<int,4>> odd_version(const std::array<int,2>& monitor_size,int number_of_windows, const Preference& pref){
    std::vector<std::array<int,4>> arrays;
    if(number_of_windows<3){
        
    }
    if(number_of_windows<4){
    switch(pref){
        case Preference::VERTICAL:
            
        break;
        case Preference::MOSTLY_VERTICAL:

        break;
        case Preference::HORIZONTAL:

        break;
        case Preference::MOSTLY_HORIZONTAL:

        break;
        default:
    }
    }
    else{
    switch(pref){
        case Preference::VERTICAL:
            
        break;
        case Preference::MOSTLY_VERTICAL:

        break;
        case Preference::HORIZONTAL:

        break;
        case Preference::MOSTLY_HORIZONTAL:

        break;
        default:
    };
    }
    return arrays;
};

std::vector<std::array<int,4>> even_version(const std::array<int,2>& monitor_size,int number_of_windows, const Preference& pref){
    std::vector<std::array<int,4>> arrays;
    if(number_of_windows<4){
    switch(pref){
        case Preference::VERTICAL:
            
        break;
        case Preference::MOSTLY_VERTICAL:

        break;
        case Preference::HORIZONTAL:

        break;
        case Preference::MOSTLY_HORIZONTAL:

        break;
        default:
    }
    }
    else{
    switch(pref){
        case Preference::VERTICAL:
            
        break;
        case Preference::MOSTLY_VERTICAL:

        break;
        case Preference::HORIZONTAL:

        break;
        case Preference::MOSTLY_HORIZONTAL:

        break;
        default:
    };
    }
    return arrays;
};

std::vector<std::array<int,4>> compute_optimal_arrangement(const std::array<int,2>& monitor_size,int number_of_windows, const Preference& pref){
    std::vector<std::array<int,4>> arrangement;
    arrangement.resize(number_of_windows);
    return (number_of_windows % 2 == 0) ? even_version(monitor_size,number_of_windows,pref) : odd_version(monitor_size,number_of_windows,pref);
}

int main(){
    std::array<int,2> monitor_size = {1000,1000};

    return 0;
}