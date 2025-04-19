#ifndef ACTION_H
#define ACTION_H

#include <vector>
#include <array>

/* -----------------------------------------
    * Modified by Ian Kersz Amaral - 00338368 - 2025/1
   ----------------------------------------- */

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

    float tp;
    float ti;
    float td;

    class Twiddler
    {
        enum TwiddleParams
        {
            TWIDDLE_P = 0,
            TWIDDLE_I = 1,
            TWIDDLE_D = 2,
            TWIDDLE_COUNT
        };
        TwiddleParams param_idx;

        float best_error;
        std::array<float, TWIDDLE_COUNT> potential;

        enum TwiddleSteps
        {
            TWIDDLE_UP,
            TWIDDLE_DOWN,
            TWIDDLE_REDUCE
        };
        TwiddleSteps curr_step;

        float total_error;

        int last_change;

    public:
        Twiddler() = default;
        Twiddler(const float tp, const float ti, const float td);
        void twiddlePID(const float cte, float &tp, float &ti, float &td);

        float get_pp() const { return potential[TWIDDLE_P]; }
        float get_pi() const { return potential[TWIDDLE_I]; }
        float get_pd() const { return potential[TWIDDLE_D]; }
    };

    Twiddler twiddle;
};

#endif // ACTION_H
