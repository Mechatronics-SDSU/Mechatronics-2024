/* 
 * @author Zix
 * Organizes massive amount of declarations I need for the control system
 */

#include "control_interface.hpp"

Interface::matrix_t Interface::percy_thrust_mapper = Interface::matrix_t 
{
    { 0, -1, -1,  0,  0,  1},
    {.9,  0,  0, .9, .9,  0},
    { 0,  1, -1,  0,  0,  1},
    {.9,  0,  0, .9,-.9,  0},
    { 0,  1,  1,  0,  0,  1},
    { 1,  0,  0, -1, -1,  0},
    { 0, -1,  1,  0,  0,  1},
    { 1,  0,  0, -1,  1,  0}
};

Interface::matrix_t Interface::junebug_thrust_mapper = Interface::matrix_t 
{
    {-1,  0,  0,  1,  0,  0},                   
    { 1,  0,  0,  1,  0,  0},
};


/* Defines Possible Commands to Be Given to the PID Controller */

Interface::desired_state_t Movements::turn(float degree)
{
    return desired_state_t{degree,0,0,0,0,0};
}

Interface::desired_state_t Movements::pitch(float degree)
{
    return desired_state_t{0,degree,0,0,0,0};
}

Interface::desired_state_t Movements::roll(float degree)
{
    return desired_state_t{0,0,degree,0,0,0};
}

Interface::desired_state_t Movements::move(float degree)
{
    return desired_state_t{0,0,0,degree,0,0};
}

Interface::desired_state_t Movements::translate(float degree)
{
    return desired_state_t{0,0,0,0,degree,0};
}

Interface::desired_state_t Movements::levitate(float degree)
{
    return desired_state_t{0,0,0,0,0,degree};
}


/* All Translator Functions take an idea and translate it into a series of commands to add to mediator queue */

Interface::command_vector_t Translator::turn(float degree)
{
    Interface::Command command1;
    command1.function.transform = &Movements::turn;
    command1.params.degree = degree;
    return command_vector_t{command1};
}

Interface::command_vector_t Translator::roll(float degree)
{
    Interface::Command command1;
    command1.function.transform = &Movements::roll;
    command1.params.degree = degree;
    return command_vector_t{command1};
}

Interface::command_vector_t Translator::pitch(float degree)
{
    Interface::Command command1;
    command1.function.transform = &Movements::pitch;
    command1.params.degree = degree;
    return command_vector_t{command1};
}

Interface::command_vector_t Translator::move(float degree)
{
    Interface::Command command1;
    command1.function.transform = &Movements::move;
    command1.params.degree = degree;
    return command_vector_t{command1};
}

Interface::command_vector_t Translator::translate(float degree)
{
    Interface::Command command1;
    command1.function.transform = &Movements::translate;
    command1.params.degree = degree;
    return command_vector_t{command1};
}

Interface::command_vector_t Translator::levitate(float degree)
{
    Interface::Command command1;
    command1.function.transform = &Movements::levitate;
    command1.params.degree = degree;
    return command_vector_t{command1};
}

Interface::command_vector_t Translator::relativePoint(float x, float y)
{
    float point_angle_radians = atan(x / y);
    float point_angle_degrees = point_angle_radians * (180/PI);
    if (y < 0)
    {
        point_angle_degrees += 180;
    }
    float point_distance_meters = sqrt(pow(x,2) + pow(y,2));

    Interface::Command command1;
    command1.function.transform = &Movements::turn;
    command1.params.degree = point_angle_degrees;

    Interface::Command command2;
    command2.function.transform = &Movements::move;
    command2.params.degree = point_distance_meters;

    return command_vector_t{command1, command2};
}

Interface::command_vector_t Translator::absolutePoint(float x, float y)
{
    float point_angle_radians = atan(y / x);
    float point_angle_degrees = point_angle_radians * (180/PI);
    float point_distance_meters = sqrt(pow(x,2) + pow(y,2));

    Interface::Command command1;
    command1.function.transform = &Movements::turn;
    command1.params.degree = point_angle_degrees;

    Interface::Command command2;
    command2.function.transform = &Movements::move;
    command2.params.degree = point_distance_meters;

    return command_vector_t{command1, command2};
}

Interface::command_vector_t Translator::pureRelativePoint(float x, float y)
{
    float point_angle_radians = atan(y / x);
    float point_angle_degrees = point_angle_radians * (180/PI);
    float point_distance_meters = sqrt(pow(x,2) + pow(y,2));

    Interface::Command command1;
    command1.function.transform = &Movements::turn;
    command1.params.degree = point_angle_degrees;

    Interface::Command command2;
    command2.function.transform = &Movements::move;
    command2.params.degree = point_distance_meters;

    return command_vector_t{command1, command2};
}

Interface::command_vector_t Translator::pureAbsolutePoint(float x, float y)
{
    float point_angle_radians = atan(y / x);
    float point_angle_degrees = point_angle_radians * (180/PI);
    float point_distance_meters = sqrt(pow(x,2) + pow(y,2));

    Interface::Command command1;
    command1.function.transform = &Movements::turn;
    command1.params.degree = point_angle_degrees;

    Interface::Command command2;
    command2.function.transform = &Movements::move;
    command2.params.degree = point_distance_meters;

    return command_vector_t{command1, command2};
}



