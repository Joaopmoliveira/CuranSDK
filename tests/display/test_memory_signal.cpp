#include "userinterface/widgets/Signal.h"

enum InterpreterStatus : size_t
{
    MOUSE_CLICKED_LEFT_EVENT = 1 << 1,
    MOUSE_CLICKED_LEFT = 1 << 2,
    MOUSE_CLICKED_RIGHT_EVENT = 1 << 3,
    MOUSE_CLICKED_RIGHT = 1 << 4,
    MOUSE_UNCLICK_LEFT_EVENT = 1 << 5,
    MOUSE_UNCLICK_RIGHT_EVENT = 1 << 6,
    MOUSE_MOVE_EVENT = 1 << 9,
    MOUSE_MOVE = 1 << 10,
    SCROLL_EVENT = 1 << 11,
    SCROLL = 1 << 12,
    OUTSIDE_ALLOCATED_AREA = 1 << 13,
    INSIDE_ALLOCATED_AREA = 1 << 14,
    ALLOCATED_AREA_EVENT = 1 << 15,
    OUTSIDE_FIXED_AREA = 1 << 16,
    INSIDE_FIXED_AREA = 1 << 17,
    FIXED_AREA_EVENT = 1 << 18,
    ITEM_DROPPED_EVENT = 1 << 19
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
    size_t current_status = ~MOUSE_CLICKED_LEFT | ~MOUSE_CLICKED_RIGHT | OUTSIDE_ALLOCATED_AREA | INSIDE_ALLOCATED_AREA;
    std::function<bool(double, double)> allocated_area;
    std::function<bool(double, double)> size_area;
public:
    SignalInterpreter(std::function<bool(double, double)> in_allocated_area, std::function<bool(double, double)> in_size_area)
    {
    }

    void process(curan::ui::Signal current_signal)
    {
        if (current_status & ~MOUSE_CLICKED_LEFT) // if previous state was unclicked and now its clicked left
            current_status |= ~MOUSE_CLICKED_LEFT | MOUSE_CLICKED_LEFT_EVENT;
        if (current_status & ~MOUSE_CLICKED_RIGHT) // if previous state was unclicked and now its clicked left
            current_status |= ~MOUSE_CLICKED_RIGHT | MOUSE_CLICKED_RIGHT_EVENT;
    };

    size_t status(){
        return current_status;
    }
};

int main(){
    SkRect rectangle_outside = SkRect::MakeXYWH(50,50,100,100);
    SkRect rectangle_inside = SkRect::MakeXYWH(75,75,25,25);
    SignalInterpreter interpreter{[&](double x, double y){ return rectangle_outside.contains(x,y); },[&](double x, double y){ return rectangle_inside.contains(x,y);}};
    return 0;
};