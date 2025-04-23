#include "userinterface/widgets/Signal.h"
#include "utils/Overloading.h"
#include <iostream>

enum InterpreterStatus : size_t
{
    MOUSE_CLICKED_LEFT_EVENT = 1 << 1,
    MOUSE_CLICKED_LEFT = 1 << 2,
    MOUSE_CLICKED_RIGHT_EVENT = 1 << 3,
    MOUSE_CLICKED_RIGHT = 1 << 4,
    MOUSE_UNCLICK_LEFT_EVENT = 1 << 5,
    MOUSE_UNCLICK_RIGHT_EVENT = 1 << 6,
    MOUSE_MOVE_EVENT = 1 << 7, // move is always an event
    SCROLL_EVENT = 1 << 8,     // scroll is always an event
    OUTSIDE_ALLOCATED_AREA = 1 << 9,
    INSIDE_ALLOCATED_AREA = 1 << 10,
    ENTERED_ALLOCATED_AREA_EVENT = 1 << 11,
    LEFT_ALLOCATED_AREA_EVENT = 1 << 12,
    OUTSIDE_FIXED_AREA = 1 << 13,
    INSIDE_FIXED_AREA = 1 << 14,
    LEFT_FIXED_AREA_EVENT = 1 << 15,
    ENTERED_FIXED_AREA_EVENT = 1 << 16,
    ITEM_DROPPED_EVENT = 1 << 17,
    KEY_EVENT = 1 << 18,
    HEART_BEAT = 1 << 19
};

/*
The signal interpreter allows us to retain state
and impose logic about how to system should behave when
transitions between these states happen. This should
compact most logic of widgets, since all state is
computed inside this interpreter
*/
class SignalInterpreter
{
    size_t current_status = OUTSIDE_ALLOCATED_AREA | OUTSIDE_FIXED_AREA;
    bool long_format = false;

    double x_last_move = 0.0;
    double y_last_move = 0.0;

    double x_last_press = 0.0;
    double y_last_press = 0.0;

public:
    SignalInterpreter()
    {
    }
private:
    inline void shutoff_oneoff_events()
    {
        // all events are one off, meaning that when we receive a new signal,
        // we must shut them off so that we guarantee that the class does not
        // repeat behavior
        current_status &= ~HEART_BEAT;
        current_status &= ~MOUSE_CLICKED_LEFT_EVENT;
        current_status &= ~MOUSE_CLICKED_RIGHT_EVENT;
        current_status &= ~ITEM_DROPPED_EVENT;
        current_status &= ~LEFT_FIXED_AREA_EVENT;
        current_status &= ~ENTERED_FIXED_AREA_EVENT;
        current_status &= ~ENTERED_ALLOCATED_AREA_EVENT;
        current_status &= ~LEFT_ALLOCATED_AREA_EVENT;
        current_status &= ~SCROLL_EVENT;
        current_status &= ~MOUSE_MOVE_EVENT;
        current_status &= ~MOUSE_UNCLICK_RIGHT_EVENT;
        current_status &= ~MOUSE_UNCLICK_LEFT_EVENT;
        current_status &= ~KEY_EVENT;
    }

    template<typename allocated>
    inline void allocated_area_logic(allocated&& check_allocated,const double x, const double y)
    {
        if (check_allocated(x, y)) // if inside allocated area
        {
            if (current_status & OUTSIDE_ALLOCATED_AREA)
            {
                current_status |= ENTERED_ALLOCATED_AREA_EVENT;
            }
            current_status &= ~OUTSIDE_ALLOCATED_AREA;
            current_status |= INSIDE_ALLOCATED_AREA; 
        }
        else if (current_status & INSIDE_ALLOCATED_AREA)
        {
            current_status &= ~INSIDE_ALLOCATED_AREA;
            current_status |= LEFT_ALLOCATED_AREA_EVENT | OUTSIDE_ALLOCATED_AREA;
        }
    }

    template<typename fixed>
    inline void fixed_area_logic(fixed&& check_fixed,const double x,const double y)
    {
        if (check_fixed(x, y)) // if inside allocated area
        {
            if (current_status & OUTSIDE_FIXED_AREA)
            {
                current_status |= ENTERED_FIXED_AREA_EVENT;
            }
            current_status &= ~OUTSIDE_FIXED_AREA;
            current_status |= INSIDE_FIXED_AREA; 
        }
        else if (current_status & INSIDE_FIXED_AREA)
        {
            current_status &= ~INSIDE_FIXED_AREA;
            current_status |= LEFT_FIXED_AREA_EVENT | OUTSIDE_FIXED_AREA;
        }
    }
public:
    template<typename allocated,typename fixed>
    void process(allocated&& inside_allocated,fixed && inside_fixed,curan::ui::Signal current_signal)
    {
        std::visit(curan::utilities::overloaded{[&](curan::ui::Empty arg)
                                                {
                                                    current_status |= HEART_BEAT;
                                                },
                                                [&](curan::ui::Move arg)
                                                {
                                                    shutoff_oneoff_events();
                                                    allocated_area_logic(std::forward<allocated>(inside_allocated),arg.xpos,arg.ypos);
                                                    fixed_area_logic(std::forward<fixed>(inside_fixed),arg.xpos,arg.ypos);
                                                    current_status |= MOUSE_MOVE_EVENT;
                                                    x_last_move = arg.xpos;
                                                    y_last_move = arg.ypos;
                                                },
                                                [&](curan::ui::Press arg)
                                                {
                                                    shutoff_oneoff_events();
                                                    allocated_area_logic(std::forward<allocated>(inside_allocated),arg.xpos,arg.ypos);
                                                    fixed_area_logic(std::forward<fixed>(inside_fixed),arg.xpos,arg.ypos);
                                                    if (!(current_status & MOUSE_CLICKED_LEFT))
                                                    {
                                                        current_status |= MOUSE_CLICKED_LEFT_EVENT | MOUSE_CLICKED_LEFT;
                                                    }
                                                    x_last_press = arg.xpos;
                                                    y_last_press = arg.ypos;
                                                },
                                                [&](curan::ui::Scroll arg)
                                                {
                                                    shutoff_oneoff_events();
                                                    allocated_area_logic(std::forward<allocated>(inside_allocated),arg.xpos,arg.ypos);
                                                    fixed_area_logic(std::forward<fixed>(inside_fixed),arg.xpos,arg.ypos);
                                                    current_status |= SCROLL_EVENT;
                                                },
                                                [&](curan::ui::Unpress arg)
                                                {
                                                    shutoff_oneoff_events();
                                                    allocated_area_logic(std::forward<allocated>(inside_allocated),arg.xpos,arg.ypos);
                                                    fixed_area_logic(std::forward<fixed>(inside_fixed),arg.xpos,arg.ypos);
                                                    if ((current_status & MOUSE_CLICKED_LEFT)){
                                                        current_status |= MOUSE_UNCLICK_LEFT_EVENT;
                                                        current_status &= ~MOUSE_CLICKED_LEFT;      
                                                    }
                                                },
                                                [&](curan::ui::Key arg)
                                                {
                                                    shutoff_oneoff_events();
                                                    current_status |= KEY_EVENT;
                                                },
                                                [&](curan::ui::ItemDropped arg)
                                                {
                                                    shutoff_oneoff_events();
                                                    current_status |= ITEM_DROPPED_EVENT;
                                                }},
                   current_signal);
    };

    inline size_t status()
    {
        return current_status;
    }

    inline void set_format(bool val){
        long_format = val;
    }

    inline std::pair<double,double> last_press() const {
        return std::make_pair(x_last_press,y_last_press);
    }

    inline std::pair<double,double> last_move() const {
        return std::make_pair(x_last_move,y_last_move);
    }

    friend std::ostream &operator<<(std::ostream &o, const SignalInterpreter &val)
    {
        
        o << "status: " << std::bitset<sizeof(size_t) * 8>{val.current_status} << std::endl;

        if(!val.long_format)
            return o;

        if(val.current_status & MOUSE_CLICKED_LEFT_EVENT){
            o << "MOUSE_CLICKED_LEFT_EVENT\n";
        }

        if(val.current_status & MOUSE_CLICKED_LEFT){
            o << "MOUSE_CLICKED_LEFT\n";
        }

        if(val.current_status & MOUSE_CLICKED_RIGHT_EVENT){
            o << "MOUSE_CLICKED_RIGHT_EVENT\n";
        }

        if(val.current_status & MOUSE_CLICKED_RIGHT){
            o << "MOUSE_CLICKED_RIGHT\n";
        }

        if(val.current_status & MOUSE_UNCLICK_LEFT_EVENT){
            o << "MOUSE_UNCLICK_LEFT_EVENT\n";
        }

        if(val.current_status & MOUSE_UNCLICK_RIGHT_EVENT){
            o << "MOUSE_UNCLICK_RIGHT_EVENT\n";
        }

        if(val.current_status & MOUSE_MOVE_EVENT){
            o << "MOUSE_MOVE_EVENT\n";
        }

        if(val.current_status & SCROLL_EVENT){
            o << "SCROLL_EVENT\n";
        }

        if(val.current_status & OUTSIDE_ALLOCATED_AREA){
            o << "OUTSIDE_ALLOCATED_AREA\n";
        }

        if(val.current_status & INSIDE_ALLOCATED_AREA){
            o << "INSIDE_ALLOCATED_AREA\n";
        }

        if(val.current_status & ENTERED_ALLOCATED_AREA_EVENT){
            o << "ENTERED_ALLOCATED_AREA_EVENT\n";
        }

        if(val.current_status & LEFT_ALLOCATED_AREA_EVENT){
            o << "LEFT_ALLOCATED_AREA_EVENT\n";
        }

        if(val.current_status & OUTSIDE_FIXED_AREA){
            o << "OUTSIDE_FIXED_AREA\n";
        }

        if(val.current_status & INSIDE_FIXED_AREA){
            o << "INSIDE_FIXED_AREA\n";
        }

        if(val.current_status & LEFT_FIXED_AREA_EVENT){
            o << "LEFT_FIXED_AREA_EVENT\n";
        }

        if(val.current_status & ENTERED_FIXED_AREA_EVENT){
            o << "ENTERED_FIXED_AREA_EVENT\n";
        }

        if(val.current_status & ITEM_DROPPED_EVENT){
            o << "ITEM_DROPPED_EVENT\n";
        }

        if(val.current_status & KEY_EVENT){
            o << "KEY_EVENT\n";
        }

        if(val.current_status & HEART_BEAT){
            o << "HEART_BEAT\n";
        }
        return o;
    }
};

int main()
{
    SkRect rectangle_outside = SkRect::MakeXYWH(50, 50, 100, 100);
    SkRect rectangle_inside = SkRect::MakeXYWH(75, 75, 25, 25);
    SignalInterpreter interpreter{};

    interpreter.set_format(true);
    auto check_inside_size = [&](double x, double y)
                                  { return rectangle_inside.contains(x, y); };
    auto check_allocated_area = [&](double x, double y)
                                  { return rectangle_outside.contains(x, y); };
    std::cout << interpreter << std::endl;
    interpreter.process(check_allocated_area,check_inside_size,curan::ui::Move{1,1});
    std::cout << interpreter << std::endl;
    interpreter.process(check_allocated_area,check_inside_size,curan::ui::Move{54,54});
    std::cout << interpreter << std::endl;
    interpreter.process(check_allocated_area,check_inside_size,curan::ui::Move{54,54});
    std::cout << interpreter << std::endl;
    interpreter.process(check_allocated_area,check_inside_size,curan::ui::Move{77,77});
    std::cout << interpreter << std::endl;
    interpreter.process(check_allocated_area,check_inside_size,curan::ui::Move{77,77});
    std::cout << interpreter << std::endl;
    interpreter.process(check_allocated_area,check_inside_size,curan::ui::Press{curan::ui::Press::click::LEFT,77,77});
    std::cout << interpreter << std::endl;
    interpreter.process(check_allocated_area,check_inside_size,curan::ui::Press{curan::ui::Press::click::LEFT,77,77});
    std::cout << interpreter << std::endl;
    interpreter.process(check_allocated_area,check_inside_size,curan::ui::Move{77,77});
    std::cout << interpreter << std::endl;
    interpreter.process(check_allocated_area,check_inside_size,curan::ui::Unpress{curan::ui::Unpress::click::LEFT,77,77});
    std::cout << interpreter << std::endl;
    interpreter.process(check_allocated_area,check_inside_size,curan::ui::Unpress{curan::ui::Unpress::click::LEFT,77,77});
    std::cout << interpreter << std::endl;
    interpreter.process(check_allocated_area,check_inside_size,curan::ui::Move{54,54});
    std::cout << interpreter << std::endl;
    interpreter.process(check_allocated_area,check_inside_size,curan::ui::Move{1,1});
    std::cout << interpreter << std::endl;

    std::cout << "size of signal interpreter = " << sizeof(SignalInterpreter) << std::endl;

    return 0;
};