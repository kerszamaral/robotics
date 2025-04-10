#include "Action.h"

#include "Utils.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <numeric>
#include <ostream>

Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;
    cte_sum = 0.0f;
    cte_last = 0.0f;
    
    tp = 0.2f;
    ti = 0.0012f;
    td = 3.5f;

    twiddle = Twiddler(tp, ti, td);
}

static constexpr auto twiddle_start = 0.1f;

Action::Twiddler::Twiddler(const float tp, const float ti, const float td)
{
    param_idx = TWIDDLE_P;
    best_error = std::numeric_limits<decltype(best_error)>::max();
    potential = {twiddle_start*tp, twiddle_start*ti, twiddle_start*td};
    curr_step = TWIDDLE_UP;
    total_error = 0.0f;
    last_change = 0;
}

void Action::avoidObstacles(std::vector<float> lasers, std::vector<float> sonars)
{
    (void) sonars;
    static constexpr auto middle = 2;
    static constexpr auto middle_offset = 5;

    // Break the lasers into parts
    const auto laser_middle = lasers.size()/middle;
    const auto start_front_laser = laser_middle - (lasers.size()/middle_offset);
    // Get the minimum distance in the front cone
    const auto min_front_dist = *std::min_element(lasers.begin()+start_front_laser, lasers.end()-start_front_laser);


    static constexpr auto dist_threshold = 1.2f;
    static constexpr auto wander_lin_velocity = 0.3f;
    static constexpr auto avoid_ang_velocity = 0.1f;
    static constexpr auto side_threshold = 0.01f;
    // If the minimum distance is below the threshold, we need to avoid
    if (min_front_dist < dist_threshold)
    {
        linVel = 0.0f;
        // Get the minimum distance in the left and right cones
        const auto &left_min = *std::min_element(lasers.begin(), lasers.begin()+laser_middle);
        const auto &right_min = *std::min_element(lasers.begin()+laser_middle, lasers.end());

        // Decide the direction to turn
        const auto ang_size = right_min - left_min > side_threshold ? -1 : 1;
        angVel = ang_size * avoid_ang_velocity;
        // std::cout << "Min Dist: " << min_front_dist << std::endl;
    }
    else
    {
        // If the distance is above the threshold, we can wander
        linVel = wander_lin_velocity;
        angVel = 0.0f;
    }
}

// Gotten from https://tfetimes.com/c-map-range/ 
template<typename tVal>
tVal map_value(std::pair<tVal,tVal> a, std::pair<tVal, tVal> b, tVal inVal)
{
	tVal inValNorm = inVal - a.first;
	tVal aUpperNorm = a.second - a.first;
	tVal normPosition = inValNorm / aUpperNorm;

	tVal bUpperNorm = b.second - b.first;
	tVal bValNorm = normPosition * bUpperNorm;
	tVal outVal = b.first + bValNorm;

	return outVal;
}

void Action::keepAsFarthestAsPossibleFromWalls(std::vector<float> lasers, std::vector<float> sonars)
{
    (void) sonars;
    
    constexpr auto parts = 3;
    const auto laser_part = lasers.size()/parts;
    
    static constexpr auto keep_lin_velocity = 0.3f;
    static constexpr auto INTERP_VELOCITY = true;

    // Get the minimum distance in the front cone
    const auto &front_min = *std::min_element(lasers.begin()+laser_part, lasers.end()-laser_part);
    if constexpr (INTERP_VELOCITY)
    {
        static constexpr auto front_threshold = 0.9f;
        static constexpr auto min_front_threshold = 0.6f;
        
        // If distance is above the front threshold, we can keep the linear velocity
        // If distance is below the front threshold, we need to slow down
        // If distance is below the min front threshold, we need to stop
        const auto &mapped_value = map_value(
            std::make_pair(front_threshold, min_front_threshold), // Input range
            std::make_pair(keep_lin_velocity, 0.0f), // Output range
            front_min // Input value
        );
        // Clamp the mapped value to the range [0, keep_lin_velocity]
        linVel = std::clamp(mapped_value, 0.0f, keep_lin_velocity);
    }
    else
    {
        linVel = keep_lin_velocity; // Keep linear velocity constant
    }

    // Get the minimum distance in the left and right cones
    const auto &left_min = *std::min_element(lasers.begin(), lasers.begin()+laser_part);
    const auto &right_min = *std::min_element(lasers.end()-laser_part, lasers.end());


    // Calculate the cross track error, account for the distance between the lasers
    constexpr auto distance = 0.5f;
    const auto cross_track_error = right_min - left_min + distance;

    // Clamp the cross track error to the range [-1, 1], so we dont get too big values
    constexpr auto bound = 1.0f;
    const auto cte = std::clamp(cross_track_error, -bound, bound);

    // P
    const auto P = tp*cte;

    // I
    cte_sum += cte;
    const auto I = ti*cte_sum;

    //D
    const auto &deriv_of_cte = cte - cte_last;
    const auto D = td*deriv_of_cte;
    cte_last = cte;

    // Calculate the new angle based on the PID
    const auto &PID = -(P + I + D);

    angVel = PID;

    // Twiddle the PID parameters
    twiddle.twiddlePID(cte, tp, ti, td);

    constexpr auto SHOW_DEBUG = true;
    if constexpr (SHOW_DEBUG)
    {
        std::cout
            << "PID Debug: \n"
            << "F: " << front_min << " LinVel: " << linVel << "\n"
            << "L: " << left_min << " R: " << right_min << "\n"
            << "CTE: " << cte << "\n"
            << "P " << P << " I " << I << " D " << D << "\n"
            << "TP: " << tp << " TI: " << ti << " TD: " << td << "\n"
            << "PP: " << twiddle.get_pp() << " PI: " << twiddle.get_pi() << " PD: " << twiddle.get_pd() << "\n"
            << "PID: " << PID << std::endl;
    }
}


void Action::Twiddler::twiddlePID(const float cte, float &tp, float &ti, float &td)
{
    // If all potential values are below the threshold, stop twiddling
    static constexpr auto potential_threshold = 0.00001f;
    const auto &potential_sum = std::reduce(potential.begin(), potential.end());
    if (potential_sum < potential_threshold)
    {
        return;
    }

    // We dont want to twiddle every iteration, so we count and sum the errors
    static constexpr auto steps_until_recalc = 20;
    if (last_change < steps_until_recalc)
    {
        total_error += std::pow(cte, 2);
        ++last_change;
        return;
    }
    last_change = 0;


    const auto &err = total_error;
    // If the error is better tha the current best
    if (err < best_error)
    {
        // Save the best error and increase the potential, also reset the steps
        best_error = err;
        potential[param_idx] *= 1.1f;
        curr_step = TWIDDLE_UP;
    }
    total_error = 0.0f;

    // Get the current parameter to twiddle
    auto &curr_param =   param_idx == TWIDDLE_P ? tp 
                                : param_idx == TWIDDLE_I ? ti
                                : td;

    switch (curr_step)
    {
    case TWIDDLE_UP:
        // Increase the parameter based on the potential
        curr_param += potential[param_idx];
        curr_step = TWIDDLE_DOWN;
        break;
    case TWIDDLE_DOWN:
        // Subtracts the potential from the parameter, 2x because we remove the addition above
        curr_param -= 2 * potential[param_idx];
        curr_step = TWIDDLE_REDUCE;
        break;
    case TWIDDLE_REDUCE:
        // The error is worse, so we reset the parameter back and reduce the potential
        curr_param += potential[param_idx];
        potential[param_idx] *= 0.9f;
        curr_step = TWIDDLE_UP;
        // Also go to the next parameter, wrapping around
        param_idx = static_cast<TwiddleParams>((param_idx + 1) % TWIDDLE_COUNT);
        break;
    }
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

