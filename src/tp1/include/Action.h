#ifndef ACTION_H
#define ACTION_H

#include <vector>
#include <array>

enum MotionMode {MANUAL, WANDER, FARFROMWALLS};
enum MovingDirection {STOP, FRONT, BACK, LEFT, RIGHT, AUTO};

typedef struct
{
    MotionMode mode;
    MovingDirection direction;
} MotionControl;

class Action
{
public:
    Action();
    
    void manualRobotMotion(MovingDirection direction);
    void avoidObstacles(std::vector<float> lasers, std::vector<float> sonars);
    void keepAsFarthestAsPossibleFromWalls(std::vector<float> lasers, std::vector<float> sonars);

    MotionControl handlePressedKey(char key);

    void correctVelocitiesIfInvalid();
    float getLinearVelocity();
    float getAngularVelocity();

private:
    float linVel;
    float angVel;

    float cte_sum;
    float cte_last;

    std::array<float, 3> t;
    
    int twiddle_curr;
    float twiddle_best_error;
    std::array<float, 3> twiddle_dp;
    int twiddle_step;
    float twiddle_total_error;

    int twiddle_last_change;

    void twiddlePID(float cte);
};

#endif // ACTION_H
