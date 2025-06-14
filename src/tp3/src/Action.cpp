#include "Action.h"

#include "Utils.h"

/* -----------------------------------------
    * Modified by Ian Kersz Amaral - 00338368 - 2025/1
   ----------------------------------------- */

//////////////////
/// CONSTRUTOR ///
//////////////////

Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;
}

///////////////////////////////
/// FUNCOES DE MOVIMENTACAO ///
///////////////////////////////

void Action::followDirection(double angle)
{
    /// TODO:
    /// atualiza valores de velocidade linear e angular (variaveis linVel e angVel)
    /// usando controle P em funcao do angulo da direção de movimento desejada
    /// quanto maior o angulo desejado, maior a velocidade angular aplicada
    
    /// se o valor do angulo for muito grande (tanto positivo quanto negativo),
    /// fazer robô girar sobre o proprio eixo

    /// usar valores maximos baixos em linVel (ex: 0.2) e angVel (ex: 0.3)
    constexpr float maxLinVel = 0.2;
    constexpr float maxAngVel = 0.3;
    // Received angle is the difference from the robot orientation to the desired direction
    constexpr float highAngleThreshold = 50.0;

    if (std::abs(angle) > highAngleThreshold)
    {
        angVel = (angle > 0) ? maxAngVel : -maxAngVel; // Rotate in place
        linVel = 0.0;
    }
    else
    {
        angVel = angle * maxAngVel / highAngleThreshold; // Proportional control
        linVel = maxLinVel * (1.0 - std::abs(angle) / highAngleThreshold); // Reduce linear speed with angle
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

void Action::stopRobot()
{
    linVel=0.0;
    angVel=0.0;
}

///////////////////////////
/// FUNCOES AUXILIARES  ///
///////////////////////////

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
        mc.mode=EXPLORE;
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

