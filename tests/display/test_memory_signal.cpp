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
    std::function<bool(const double,const double)> allocated_area;
    std::function<bool(const double,const double)> size_area;
    bool long_format = false;

public:
    SignalInterpreter(std::function<bool(double, double)> in_allocated_area, std::function<bool(double, double)> in_size_area) : allocated_area{in_allocated_area}, size_area{in_size_area}
    {
        current_status = OUTSIDE_ALLOCATED_AREA | OUTSIDE_FIXED_AREA;
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

    inline void allocated_area_logic(const double x, const double y)
    {
        if (allocated_area(x, y)) // if inside allocated area
        {
            if (current_status & ~INSIDE_ALLOCATED_AREA)
            {
                current_status |= ENTERED_ALLOCATED_AREA_EVENT;
            }
            current_status |= INSIDE_ALLOCATED_AREA & ~OUTSIDE_ALLOCATED_AREA;
        }
        else if (current_status & INSIDE_ALLOCATED_AREA)
        {
            current_status |= LEFT_ALLOCATED_AREA_EVENT & ~INSIDE_ALLOCATED_AREA & OUTSIDE_ALLOCATED_AREA;
        }
    }

    inline void fixed_area_logic(const double x,const double y)
    {
        if (size_area(x, y)) // if inside allocated area
        {
            if (current_status & ~INSIDE_FIXED_AREA)
            {
                current_status |= ENTERED_FIXED_AREA_EVENT;
            }
            current_status |= INSIDE_FIXED_AREA;
        }
        else if (current_status & INSIDE_FIXED_AREA)
        {
            current_status |= LEFT_FIXED_AREA_EVENT;
        }
    }
public:
    void process(curan::ui::Signal current_signal)
    {

        std::visit(curan::utilities::overloaded{[this](curan::ui::Empty arg)
                                                {
                                                    current_status |= HEART_BEAT;
                                                    std::cout << std::bitset<sizeof(size_t) * 8>{current_status} << std::endl;
                                                },
                                                [this](curan::ui::Move arg)
                                                {
                                                    shutoff_oneoff_events();
                                                    allocated_area_logic(arg.xpos,arg.ypos);
                                                    fixed_area_logic(arg.xpos,arg.ypos);
                                                    current_status |= MOUSE_MOVE_EVENT;
                                                    std::cout << std::bitset<sizeof(size_t) * 8>{current_status} << std::endl;
                                                },
                                                [this](curan::ui::Press arg)
                                                {
                                                    shutoff_oneoff_events();
                                                    allocated_area_logic(arg.xpos,arg.ypos);
                                                    fixed_area_logic(arg.xpos,arg.ypos);
                                                    if (current_status & ~MOUSE_CLICKED_LEFT && current_status & ~MOUSE_CLICKED_RIGHT)
                                                    {
                                                        current_status |= MOUSE_CLICKED_LEFT_EVENT | MOUSE_CLICKED_LEFT;
                                                        current_status |= MOUSE_CLICKED_RIGHT_EVENT | MOUSE_CLICKED_RIGHT;
                                                    }
                                                    std::cout << std::bitset<sizeof(size_t) * 8>{current_status} << std::endl;
                                                },
                                                [this](curan::ui::Scroll arg)
                                                {
                                                    shutoff_oneoff_events();
                                                    allocated_area_logic(arg.xpos,arg.ypos);
                                                    fixed_area_logic(arg.xpos,arg.ypos);
                                                    current_status |= SCROLL_EVENT;
                                                    std::cout << std::bitset<sizeof(size_t) * 8>{current_status} << std::endl;
                                                },
                                                [this](curan::ui::Unpress arg)
                                                {
                                                    shutoff_oneoff_events();
                                                    allocated_area_logic(arg.xpos,arg.ypos);
                                                    fixed_area_logic(arg.xpos,arg.ypos);
                                                    current_status |= MOUSE_UNCLICK_LEFT_EVENT | MOUSE_UNCLICK_RIGHT_EVENT | (~MOUSE_CLICKED_LEFT & ~MOUSE_CLICKED_RIGHT);
                                                    std::cout << std::bitset<sizeof(size_t) * 8>{current_status} << std::endl;
                                                },
                                                [this](curan::ui::Key arg)
                                                {
                                                    shutoff_oneoff_events();
                                                    current_status |= KEY_EVENT;
                                                    std::cout << std::bitset<sizeof(size_t) * 8>{current_status} << std::endl;
                                                },
                                                [this](curan::ui::ItemDropped arg)
                                                {
                                                    shutoff_oneoff_events();
                                                    current_status |= ITEM_DROPPED_EVENT;
                                                    std::cout << std::bitset<sizeof(size_t) * 8>{current_status} << std::endl;
                                                }},
                   current_signal);
    };

    size_t status()
    {
        return current_status;
    }

    void set_format(bool val){
        long_format = val;
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
    SignalInterpreter interpreter{[&](double x, double y)
                                  { return rectangle_outside.contains(x, y); }, [&](double x, double y)
                                  { return rectangle_inside.contains(x, y); }};

    interpreter.set_format(true);
    std::cout << interpreter << std::endl;
    curan::ui::Move arg{1,1};
    interpreter.process(arg);
    std::cout << interpreter << std::endl;
    arg=curan::ui::Move{54,54};
    interpreter.process(arg);
    std::cout << interpreter << std::endl;
    return 0;
};