#include "Action.h"

#include "Utils.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <numeric>
#include <ostream>

static constexpr auto p = 0;
static constexpr auto i = 1;
static constexpr auto d = 2;

static constexpr auto twiddle_start = 0.1f;

Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;
    cte_sum = 0.0f;
    cte_last = 0.0f;

    t[p] = 0.2f;
    t[i] = 0.0012f;
    t[d] = 3.5f;

    twiddle_curr = p;
    twiddle_best_error = std::reduce(t.begin(), t.end());
    twiddle_dp = {twiddle_start, twiddle_start, twiddle_start};
    twiddle_step = 0;
    twiddle_total_error = 0.0f;
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
    (void) sonars;
    constexpr auto keep_lin_velocity = 0.3f;
    linVel = keep_lin_velocity; // Keep linear velocity constant

    constexpr auto parts = 3;
    const auto laser_part = lasers.size()/parts;

    const auto &left_min = *std::min_element(lasers.begin(), lasers.begin()+laser_part);
    const auto &right_min = *std::min_element(lasers.end()-laser_part, lasers.end());
    const auto &front_min = *std::min_element(lasers.begin()+laser_part, lasers.end()-laser_part);


    (void) front_min;
    constexpr auto distance = 0.5f;
    const auto cross_track_error = right_min - left_min + distance;

    const auto &tp = t[p];
    const auto &ti = t[i];
    const auto &td = t[d];

    // P
    constexpr auto bound = 1.0f;
    const auto cte = std::clamp(cross_track_error, -bound, bound);
    const auto P = tp*cte;

    // I
    cte_sum += cte;
    const auto I = ti*cte_sum;

    //D
    const auto &deriv_of_cte = cte - cte_last;
    const auto D = td*deriv_of_cte;
    cte_last = cte;

    const auto new_angle = -(P + I + D);

    angVel = new_angle;


    twiddlePID(cte);

    std::cout 
        << "L: " << left_min << " R: " << right_min << std::endl
        << "CTE: " << cte << std::endl
        << "P " << P << " I " << I << " D " << D << std::endl
        << "TP: " << tp << " TI: " << ti << " TD: " << td << std::endl
        << "new_angle: " << new_angle << std::endl;
}


void Action::twiddlePID(const float cte)
{
    constexpr auto twiddle_change_steps = 200;

    if (twiddle_last_change < twiddle_change_steps)
    {
        twiddle_total_error += std::pow(cte, 2);
        ++twiddle_last_change;
        return;
    }
    twiddle_last_change = 0;


    // Twiddle algorithm to tune PID parameters
    const auto sum = std::reduce(t.begin(), t.end());

    constexpr auto threshold = 0.00002f;

    if (sum < threshold)
    {
        std::cout << "Twiddle finished" << std::endl;
        return;
    }

    const auto &err = twiddle_total_error;
    if (err < twiddle_best_error)
    {
        twiddle_best_error = err;
        twiddle_dp[twiddle_curr] *= 1.1f;
        twiddle_step = 0;
    }

    if (twiddle_step == 0)
    {
        t[twiddle_curr] += twiddle_dp[twiddle_curr];

        twiddle_step = 1;
    } else if (twiddle_step == 1)
    {
        t[twiddle_curr] -= 2 * twiddle_dp[twiddle_curr];

        twiddle_step = 2;
    } else if (twiddle_step == 2)
    {
        t[twiddle_curr] += twiddle_dp[twiddle_curr];
        twiddle_dp[twiddle_curr] *= 0.9f;

        twiddle_step = 0;

        twiddle_curr = (twiddle_curr + 1) % t.size();
    }

    twiddle_total_error = 0.0f;
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

