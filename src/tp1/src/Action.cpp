#include "Action.h"

#include "Utils.h"
#include <algorithm>
#include <iostream>
#include <iterator>
#include <ostream>

Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;
}

void Action::avoidObstacles(std::vector<float> lasers, std::vector<float> sonars)
{
    (void) sonars;
    constexpr auto middle = 2;
    constexpr auto middle_offset = 5;
    const auto laser_middle = lasers.size()/middle;
    const auto start_front_laser = laser_middle - (lasers.size()/middle_offset);
    const auto min_front_dist = *std::min_element(lasers.begin()+start_front_laser, lasers.end()-start_front_laser);


    constexpr auto dist_threshold = 1.2f;
    constexpr auto wander_lin_velocity = 0.3f;
    constexpr auto avoid_ang_velocity = 0.1f;
    constexpr auto side_threshold = 0.01f;
    if (min_front_dist < dist_threshold)
    {
        linVel = 0.0f;
        const auto &left_min = *std::min_element(lasers.begin(), lasers.begin()+laser_middle);
        const auto &right_min = *std::min_element(lasers.begin()+laser_middle, lasers.end());

        const auto ang_size = right_min - left_min > side_threshold ? -1 : 1;
        angVel = ang_size * avoid_ang_velocity;
        // std::cout << "Min Dist: " << min_front_dist << std::endl;
    }
    else
    {
        linVel = wander_lin_velocity;
        angVel = 0.0f;
    }
}

void Action::keepAsFarthestAsPossibleFromWalls(std::vector<float> lasers, std::vector<float> sonars)
{
    // TODO: COMPLETAR FUNCAO





}

void Action::manualRobotMotion(MovingDirection direction)
{
    if(direction == FRONT){
        linVel= 0.5; angVel= 0.0;
    }else if(direction == BACK){
        linVel=-0.5; angVel= 0.0;
    }else if(direction == LEFT){
        linVel= 0.0; angVel= 0.5;
    }else if(direction == RIGHT){
        linVel= 0.0; angVel=-0.5;
    }else if(direction == STOP){
        linVel= 0.0; angVel= 0.0;
    }
}

void Action::correctVelocitiesIfInvalid()
{
    float b=0.38;

    float leftVel  = linVel - angVel*b/(2.0);
    float rightVel = linVel + angVel*b/(2.0);

    float VELMAX = 0.5;

    float absLeft = fabs(leftVel);
    float absRight = fabs(rightVel);

    if(absLeft>absRight){
        if(absLeft > VELMAX){
            leftVel *= VELMAX/absLeft;
            rightVel *= VELMAX/absLeft;
        }
    }else{
        if(absRight > VELMAX){
            leftVel *= VELMAX/absRight;
            rightVel *= VELMAX/absRight;
        }
    }
    
    linVel = (leftVel + rightVel)/2.0;
    angVel = (rightVel - leftVel)/b;
}

float Action::getLinearVelocity()
{
    return linVel;
}

float Action::getAngularVelocity()
{
    return angVel;
}

MotionControl Action::handlePressedKey(char key)
{
    MotionControl mc;
    mc.mode=MANUAL;
    mc.direction=STOP;

    if(key=='1'){
        mc.mode=MANUAL;
        mc.direction=STOP;
    }else if(key=='2'){
        mc.mode=WANDER;
        mc.direction=AUTO;
    }else if(key=='3'){
        mc.mode=FARFROMWALLS;
        mc.direction=AUTO;
    }else if(key=='w' or key=='W'){
        mc.mode=MANUAL;
        mc.direction = FRONT;
    }else if(key=='s' or key=='S'){
        mc.mode=MANUAL;
        mc.direction = BACK;
    }else if(key=='a' or key=='A'){
        mc.mode=MANUAL;
        mc.direction = LEFT;
    }else if(key=='d' or key=='D'){
        mc.mode=MANUAL;
        mc.direction = RIGHT;
    }else if(key==' '){
        mc.mode=MANUAL;
        mc.direction = STOP;
    }
    
    return mc;
}

