#include "rendering/SequencialLinks.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/Sphere.h"
#include <tuple>
#include <iostream>
#include <charconv>

constexpr double my_pi = 3.1415;


enum joints_representation{
    q0,
    q1,
    q2,
    q3,
    q4,
    q5,
    q6,
};

class joint_token{
    bool is_defined = false;
    joints_representation representation;

public:

    friend std::istream & operator>>(std::istream & i,joint_token & d);

    operator bool() const {
        return is_defined;
    }

    inline void set(joints_representation in_representation){
        representation = in_representation;
        is_defined = true;
    }

    std::optional<joints_representation> operator()() const {
        if(*this){
            return representation;
        }
        return std::nullopt;
    }

};

template<typename Num>
std::optional<Num> read_number(std::istream & i){
    char q;
    std::stringstream s;
    while(std::cin >> q){
        if(q=='.' || q==','){
            s << q;
            continue;
        }
           
        if(isblank(q) && q!='\n')
            continue;

        if(isdigit(q) || q=='-'){
            s << q;
            continue;
        }

        if(q=='\n'){
            return std::nullopt;
        }
            

        break;
        
    }

    std::cin.putback(q);

    std::string str = s.str();
    Num result{};
    auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), result);
 
    if (ec == std::errc())
        return result;
    else if (ec == std::errc::invalid_argument)
        return std::nullopt;   
    else if (ec == std::errc::result_out_of_range)
        return std::nullopt;   

    return std::nullopt;
}

std::istream & operator>>(std::istream & i,joint_token & d){
    char q;
    while(std::cin >> q){
        if(ispunct(q))
            continue;
        if(isblank(q) && q!='\n')
            continue;
        if(q=='q')
            break;

        return i;
    }

    auto value = read_number<size_t>(i);
    if(!value){
        return i;
    }
        
    switch(*value){
        case joints_representation::q0:
            d.set(joints_representation::q0);
        break;
        case joints_representation::q1:
            d.set(joints_representation::q1);
        break;
        case joints_representation::q2:
            d.set(joints_representation::q2);
        break;
        case joints_representation::q3:
            d.set(joints_representation::q3);
        break;
        case joints_representation::q4:
            d.set(joints_representation::q4);
        break;
        case joints_representation::q5:
            d.set(joints_representation::q5);
        break;
        case joints_representation::q6:
            d.set(joints_representation::q6);
        break;
    }
    return i;

};

struct joint_angle{
    bool is_defined = false;
    double value;

public:

    friend std::istream & operator>>(std::istream & i,joint_angle & d);

    operator bool() const {
        return is_defined;
    }

    std::optional<double> operator()() const {
        if(*this){
            return value;
        }
        return std::nullopt;
    }

    inline void set(double in){
        value = in;
        is_defined = true;
    }


};

std::istream & operator>>(std::istream & i,joint_angle & d){
    char q;
    while(std::cin >> q){
        if(isblank(q) && q!='\n')
            continue;

        if(q=='{')
            break;

        return i;
    }

    auto value = read_number<double>(i);
    if(!value)
        return i;

    while(std::cin >> q){
        if(isblank(q) && q!='\n')
            continue;

        if(q=='}')
            break;

        return i;
    }

    d.set(*value);
    return i;
}


class joint_info{
    joints_representation representation;
    double in_angle;

    public:

    joint_info(const joint_token& intoken,const joint_angle& inangle) {
        if(!intoken)
            throw std::runtime_error("token is incorrect");
        if(!inangle)
            throw std::runtime_error("token is incorrect");
        representation = *(intoken());
        in_angle = *(inangle());
    }

    inline size_t joint() const {
        return representation;
    }

    inline double angle() const {
        return in_angle;
    }
};


std::optional<joint_info> read_joint_token(std::istream & i){
    joint_token token;
    i >> token;
    if(!token){
        return std::nullopt;
    }

    joint_angle angle;
    i >> angle;
    if(!angle){
        return std::nullopt;
    }

    return joint_info{token,angle};
};

std::vector<joint_info> get_joint_tokens(std::istream & i){
    std::vector<joint_info> vals;
    while(true){
        auto joint = read_joint_token(i);
        if(!joint)
            return vals;
        vals.push_back(*joint);
    }
}

void joint_reader(vsg::ref_ptr<curan::renderable::Renderable> inrobot){
    auto robot = inrobot->cast<curan::renderable::SequencialLinks>();
    std::cout << "reading outputstream\n";
    while(true){
        auto joint_tokens = get_joint_tokens(std::cin);
        for(const auto& token : joint_tokens)
            robot->set(token.joint(), token.angle());
    }
}

int main(int argc, char **argv) {
    try {
        curan::renderable::Window::Info info;
        info.api_dump = false;
        info.display = "";
        info.full_screen = false;
        info.is_debug = false;
        info.screen_number = 0;
        info.title = "myviewer";
        curan::renderable::Window::WindowSize size{1000, 800};
        info.window_size = size;
        curan::renderable::Window window{info};

        std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/arm.json";
        curan::renderable::SequencialLinks::Info create_info;
        create_info.convetion = vsg::CoordinateConvention::Y_UP;
        create_info.json_path = robot_path;
        create_info.number_of_links = 8;
        vsg::ref_ptr<curan::renderable::Renderable> robotRenderable = curan::renderable::SequencialLinks::make(create_info);
        window << robotRenderable;

        std::thread local{[robotRenderable](){
            joint_reader(robotRenderable);
        }};
        local.detach();
        /*
        std::array<std::tuple<size_t, double>, 8> angle{{{0, 0.0}, {1,(90.0/180.0)*my_pi}, {2, 0.0}, {3, 0.0}, {4, 0.0}, {5,  (90.0/180.0)*my_pi}, {6, 0.0}, {7, 0.0}}};
        double time = 0.0;
        auto robot = robotRenderable->cast<curan::renderable::SequencialLinks>();
        for (const auto &val : angle)
            robot->set(std::get<0>(val), std::get<1>(val));
        */
        window.run();

        window.transverse_identifiers(
            [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable>>
                   &map) {
                for (auto &p : map){
                    std::cout << "Object contained: " << p.first << '\n';
                }

            });

    } catch (const std::exception& e) {
        std::cerr << "Exception thrown : " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
