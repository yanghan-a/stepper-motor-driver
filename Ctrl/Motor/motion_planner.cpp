#include "motion_planner.h"
#include "math.h"


void MotionPlanner::CurrentTracker::Init()
{
    SetCurrentAcc(context->config->ratedCurrentAcc);
}


void MotionPlanner::CurrentTracker::NewTask(int32_t _realCurrent)
{
    currentIntegral = 0;
    trackCurrent = _realCurrent;
}


void MotionPlanner::CurrentTracker::CalcSoftGoal(int32_t _goalCurrent)
{
    int32_t deltaCurrent = _goalCurrent - trackCurrent;

    if (deltaCurrent == 0)
    {
        trackCurrent = _goalCurrent;
    } else if (deltaCurrent > 0)
    {
        if (trackCurrent >= 0)
        {
            CalcCurrentIntegral(currentAcc);
            if (trackCurrent >= _goalCurrent)
            {
                currentIntegral = 0;
                trackCurrent = _goalCurrent;
            }
        } else
        {
            CalcCurrentIntegral(currentAcc);
            if ((int32_t) trackCurrent >= 0)
            {
                currentIntegral = 0;
                trackCurrent = 0;
            }
        }
    } else if (deltaCurrent < 0)
    {
        if (trackCurrent <= 0)
        {
            CalcCurrentIntegral(-currentAcc);
            if ((int32_t) trackCurrent <= (int32_t) _goalCurrent)
            {
                currentIntegral = 0;
                trackCurrent = _goalCurrent;
            }
        } else
        {
            CalcCurrentIntegral(-currentAcc);
            if ((int32_t) trackCurrent <= 0)
            {
                currentIntegral = 0;
                trackCurrent = 0;
            }
        }
    }

    goCurrent = (int32_t) trackCurrent;
}


void MotionPlanner::CurrentTracker::CalcCurrentIntegral(int32_t _current)
{
    currentIntegral += _current;
    trackCurrent += currentIntegral / context->CONTROL_FREQUENCY;
    currentIntegral = currentIntegral % context->CONTROL_FREQUENCY;
}


void MotionPlanner::CurrentTracker::SetCurrentAcc(int32_t _currentAcc)
{
    currentAcc = _currentAcc;
}


void MotionPlanner::VelocityTracker::Init()
{
    SetVelocityAcc(context->config->ratedVelocityAcc);
}


void MotionPlanner::VelocityTracker::SetVelocityAcc(int32_t _velocityAcc)
{
    velocityAcc = _velocityAcc;
}


void MotionPlanner::VelocityTracker::NewTask(int32_t _realVelocity)// give the trackVelocity real velocity
{
    velocityIntegral = 0;
    trackVelocity = _realVelocity;
}


void MotionPlanner::VelocityTracker::CalcSoftGoal(int32_t _goalVelocity)// calculate the soft goal velocity, limit the large change of velocity
{
    int32_t deltaVelocity = _goalVelocity - trackVelocity;

    if (deltaVelocity == 0)
    {
        trackVelocity = _goalVelocity;
    } else if (deltaVelocity > 0)
    {
        if (trackVelocity >= 0)
        {
            CalcVelocityIntegral(velocityAcc);
            if (trackVelocity >= _goalVelocity)// limit the result get from CalcVelocityIntegral()
            {
                velocityIntegral = 0;
                trackVelocity = _goalVelocity;
            }
        } else
        {
            CalcVelocityIntegral(velocityAcc);
            if (trackVelocity >= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    } else if (deltaVelocity < 0)
    {
        if (trackVelocity <= 0)
        {
            CalcVelocityIntegral(-velocityAcc);
            if (trackVelocity <= _goalVelocity)
            {
                velocityIntegral = 0;
                trackVelocity = _goalVelocity;
            }
        } else
        {
            CalcVelocityIntegral(-velocityAcc);
            if (trackVelocity <= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    }

    goVelocity = (int32_t) trackVelocity;
}


void MotionPlanner::VelocityTracker::CalcVelocityIntegral(int32_t _velocity)
{
    velocityIntegral += _velocity;
    trackVelocity += velocityIntegral / context->CONTROL_FREQUENCY;
    velocityIntegral = velocityIntegral % context->CONTROL_FREQUENCY;
}


void MotionPlanner::PositionTracker::Init()
{
    SetVelocityAcc(context->config->ratedVelocityAcc);

    /*
     *  Allow to locking-brake when velocity is lower than (speedLockingBrake).
     *  The best value should be (ratedMoveAcc/1000)
     */
    speedLockingBrake = context->config->ratedVelocityAcc / 1000;
}


void MotionPlanner::PositionTracker::SetVelocityAcc(int32_t value)
{
    velocityUpAcc = value;
    velocityDownAcc = value;
    quickVelocityDownAcc = 0.5f / (float) velocityDownAcc;
}


void MotionPlanner::PositionTracker::NewTask(int32_t real_location, int32_t real_speed)
{
    velocityIntegral = 0;
    trackVelocity = real_speed;
    positionIntegral = 0;
    trackPosition = real_location;
}


void MotionPlanner::PositionTracker::CalcSoftGoal(int32_t _goalPosition, int32_t _goalAcc)
{
    int32_t deltaPosition = _goalPosition - trackPosition;

    if (deltaPosition == 0)
    {
        if ((trackVelocity >= -speedLockingBrake) && (trackVelocity <= speedLockingBrake))// the velocity is in locking-brake range
        {
            velocityIntegral = 0;
            trackVelocity = 0;
            positionIntegral = 0;
        } else if (trackVelocity > 0)
        {
            CalcVelocityIntegral(-velocityDownAcc);
            if (trackVelocity <= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        } else if (trackVelocity < 0)
        {
            CalcVelocityIntegral(velocityDownAcc);
            if (trackVelocity >= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    }
    else
    {
        if (trackVelocity == 0)
        {
            if (deltaPosition > 0)
            {
                CalcVelocityIntegral(velocityUpAcc);
            } else
            {
                CalcVelocityIntegral(-velocityUpAcc);
            }
        } else if ((deltaPosition > 0) && (trackVelocity > 0))
        {
            if (trackVelocity <= context->config->ratedVelocity)
            {
                auto need_down_location = (int32_t) ((float) trackVelocity *
                                                     (float) trackVelocity *
                                                     (float) quickVelocityDownAcc);// define the point to start to slow down
                if (abs(deltaPosition) > need_down_location)
                {
                    if (trackVelocity < context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(velocityUpAcc);
                        if (trackVelocity >= context->config->ratedVelocity)
                        {
                            velocityIntegral = 0;
                            trackVelocity = context->config->ratedVelocity;
                        }
                    } else if (trackVelocity > context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(-velocityDownAcc);
                    }
                } else
                {
                    CalcVelocityIntegral(-velocityDownAcc);
                    if (trackVelocity <= 0)
                    {
                        velocityIntegral = 0;
                        trackVelocity = 0;
                    }
                }
            } else
            {
                CalcVelocityIntegral(-velocityDownAcc);
                if (trackVelocity <= 0)
                {
                    velocityIntegral = 0;
                    trackVelocity = 0;
                }
            }
        } else if ((deltaPosition < 0) && (trackVelocity < 0))
        {
            if (trackVelocity >= -context->config->ratedVelocity)
            {
                auto need_down_location = (int32_t) ((float) trackVelocity *
                                                     (float) trackVelocity *
                                                     (float) quickVelocityDownAcc);
                if (abs(deltaPosition) > need_down_location)
                {
                    if (trackVelocity > -context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(-velocityUpAcc);
                        if (trackVelocity <= -context->config->ratedVelocity)
                        {
                            velocityIntegral = 0;
                            trackVelocity = -context->config->ratedVelocity;
                        }
                    } else if (trackVelocity < -context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(velocityDownAcc);
                    }
                } else
                {
                    CalcVelocityIntegral(velocityDownAcc);
                    if (trackVelocity >= 0)
                    {
                        velocityIntegral = 0;
                        trackVelocity = 0;
                    }
                }
            } else
            {
                CalcVelocityIntegral(velocityDownAcc);
                if (trackVelocity >= 0)
                {
                    velocityIntegral = 0;
                    trackVelocity = 0;
                }
            }
        } else if ((deltaPosition < 0) && (trackVelocity > 0))
        {
            CalcVelocityIntegral(-velocityDownAcc);
            if (trackVelocity <= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        } else if (((deltaPosition > 0) && (trackVelocity < 0)))
        {
            CalcVelocityIntegral(velocityDownAcc);
            if (trackVelocity >= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    }

    CalcPositionIntegral(trackVelocity);

    go_location = (int32_t) trackPosition;
    go_velocity = (int32_t) trackVelocity;
}


void MotionPlanner::PositionTracker::CalcPositionIntegral(int32_t value)
{
    positionIntegral += value ;
    trackPosition += positionIntegral / context->CONTROL_FREQUENCY;
    positionIntegral = positionIntegral % context->CONTROL_FREQUENCY;
}


void MotionPlanner::PositionTracker::CalcVelocityIntegral(int32_t value)
{
    velocityIntegral += value;
    trackVelocity += velocityIntegral / context->CONTROL_FREQUENCY;
    velocityIntegral = velocityIntegral % context->CONTROL_FREQUENCY;
}


void MotionPlanner::PositionInterpolator::Init()
{
    // SetVelocityAcc(context->config->ratedVelocityAcc);
    //
    // /*
    //  *  Allow to locking-brake when velocity is lower than (speedLockingBrake).
    //  *  The best value should be (ratedMoveAcc/1000)
    //  */
    // speedLockingBrake = context->config->ratedVelocityAcc / 1000;
}

void MotionPlanner::PositionInterpolator::SetVelocityAcc(int32_t value)
{
    // velocityUpAcc = value;
    // velocityDownAcc = value;
    // quickVelocityDownAcc = 0.5f / (float) velocityDownAcc;
}

void MotionPlanner::PositionInterpolator::NewTask(int32_t _realPosition, int32_t _realVelocity)
{
    // // recordPosition = _realPosition;
    // // recordPositionLast = _realPosition;
    // // estPosition = _realPosition;
    // // estVelocity = _realVelocity;
    //
    // velocityIntegral = 0;
    // trackVelocity = _realVelocity;
    // trackVelocityLast = _realVelocity;
    // positionIntegral = 0;
    // trackPosition = _realPosition;
}


void MotionPlanner::PositionInterpolator::CalcSoftGoal(int32_t _goalPosition)// this function needs to be rewriten
{
    // // recordPositionLast = recordPosition;
    // // recordPosition = _goalPosition;
    // //
    // // estPositionIntegral += (((recordPosition - recordPositionLast) * context->CONTROL_FREQUENCY)
    // //                         + ((estVelocity << 6) - estVelocity));
    // // estVelocity = estPositionIntegral >> 6;
    // // estPositionIntegral -= (estVelocity << 6);
    // //
    // // estPosition = recordPosition;
    // //
    // // goPosition = estPosition;
    // // goVelocity = estVelocity;
    //    int32_t deltaPosition = _goalPosition - trackPosition;
    //
    // if (deltaPosition == 0)
    // {
    //     if ((trackVelocity >= -speedLockingBrake) && (trackVelocity <= speedLockingBrake))// the velocity is in locking-brake range
    //     {
    //         velocityIntegral = 0;
    //         trackVelocity = 0;
    //         positionIntegral = 0;
    //     } else if (trackVelocity > 0)
    //     {
    //         CalcVelocityIntegral(-velocityDownAcc);
    //         if (trackVelocity <= 0)
    //         {
    //             velocityIntegral = 0;
    //             trackVelocity = 0;
    //         }
    //     } else if (trackVelocity < 0)
    //     {
    //         CalcVelocityIntegral(velocityDownAcc);
    //         if (trackVelocity >= 0)
    //         {
    //             velocityIntegral = 0;
    //             trackVelocity = 0;
    //         }
    //     }
    // }
    // else
    // {
    //     if (trackVelocity == 0)
    //     {
    //         if (deltaPosition > 0)
    //         {
    //             CalcVelocityIntegral(velocityUpAcc);
    //         } else
    //         {
    //             CalcVelocityIntegral(-velocityUpAcc);
    //         }
    //     } else if ((deltaPosition > 0) && (trackVelocity > 0))
    //     {
    //         if (trackVelocity <= context->config->ratedVelocity)
    //         {
    //             auto need_down_location = (int32_t) ((float) trackVelocity *
    //                                                  (float) trackVelocity *
    //                                                  (float) quickVelocityDownAcc);// define the point to start to slow down
    //             if (abs(deltaPosition) > need_down_location)
    //             {
    //                 if (trackVelocity < context->config->ratedVelocity)
    //                 {
    //                     CalcVelocityIntegral(velocityUpAcc);
    //                     if (trackVelocity >= context->config->ratedVelocity)
    //                     {
    //                         velocityIntegral = 0;
    //                         trackVelocity = context->config->ratedVelocity;
    //                     }
    //                 } else if (trackVelocity > context->config->ratedVelocity)
    //                 {
    //                     CalcVelocityIntegral(-velocityDownAcc);
    //                 }
    //             } else
    //             {
    //                 CalcVelocityIntegral(-velocityDownAcc);
    //                 if (trackVelocity <= 0)
    //                 {
    //                     velocityIntegral = 0;
    //                     trackVelocity = 0;
    //                 }
    //             }
    //         } else
    //         {
    //             CalcVelocityIntegral(-velocityDownAcc);
    //             if (trackVelocity <= 0)
    //             {
    //                 velocityIntegral = 0;
    //                 trackVelocity = 0;
    //             }
    //         }
    //     } else if ((deltaPosition < 0) && (trackVelocity < 0))
    //     {
    //         if (trackVelocity >= -context->config->ratedVelocity)
    //         {
    //             auto need_down_location = (int32_t) ((float) trackVelocity *
    //                                                  (float) trackVelocity *
    //                                                  (float) quickVelocityDownAcc);
    //             if (abs(deltaPosition) > need_down_location)
    //             {
    //                 if (trackVelocity > -context->config->ratedVelocity)
    //                 {
    //                     CalcVelocityIntegral(-velocityUpAcc);
    //                     if (trackVelocity <= -context->config->ratedVelocity)
    //                     {
    //                         velocityIntegral = 0;
    //                         trackVelocity = -context->config->ratedVelocity;
    //                     }
    //                 } else if (trackVelocity < -context->config->ratedVelocity)
    //                 {
    //                     CalcVelocityIntegral(velocityDownAcc);
    //                 }
    //             } else
    //             {
    //                 CalcVelocityIntegral(velocityDownAcc);
    //                 if (trackVelocity >= 0)
    //                 {
    //                     velocityIntegral = 0;
    //                     trackVelocity = 0;
    //                 }
    //             }
    //         } else
    //         {
    //             CalcVelocityIntegral(velocityDownAcc);
    //             if (trackVelocity >= 0)
    //             {
    //                 velocityIntegral = 0;
    //                 trackVelocity = 0;
    //             }
    //         }
    //     } else if ((deltaPosition < 0) && (trackVelocity > 0))
    //     {
    //         CalcVelocityIntegral(-velocityDownAcc);
    //         if (trackVelocity <= 0)
    //         {
    //             velocityIntegral = 0;
    //             trackVelocity = 0;
    //         }
    //     } else if (((deltaPosition > 0) && (trackVelocity < 0)))
    //     {
    //         CalcVelocityIntegral(velocityDownAcc);
    //         if (trackVelocity >= 0)
    //         {
    //             velocityIntegral = 0;
    //             trackVelocity = 0;
    //         }
    //     }
    // }
    //
    // CalcPositionIntegral(trackVelocity);
    //
    // go_location = (int32_t) trackPosition;
    // go_velocity = (int32_t) trackVelocity;
}

void MotionPlanner::PositionInterpolator::CalcPositionIntegral(int32_t value)
{
    // positionIntegral += value ;
    // trackPosition += positionIntegral / context->CONTROL_FREQUENCY;
    // positionIntegral = positionIntegral % context->CONTROL_FREQUENCY;
}


void MotionPlanner::PositionInterpolator::CalcVelocityIntegral(int32_t value)
{
    // trackVelocityLast = trackVelocity;
    // velocityIntegral += value;
    // trackVelocity += velocityIntegral / context->CONTROL_FREQUENCY;
    // velocityIntegral = velocityIntegral % context->CONTROL_FREQUENCY;
}

void MotionPlanner::TrajectoryTracker::SetSlowDownVelocityAcc(int32_t value)
{
    // velocityDownAcc = value;
    // quickVelocityDownAcc = 0.5f / (float)value;
}


void MotionPlanner::TrajectoryTracker::NewTask(int32_t real_location, int32_t real_speed, int64_t real_acceleration)
{
    // updateTime = 0;
    // overtimeFlag = false;
    // dynamicVelocityAccRemainder = 0;
    // velocityNow = real_speed;
    // velocityLast = real_speed;
    // accelerationNow = real_acceleration;
    // velovityNowRemainder = 0;
    // positionNow = real_location;
    // positionLast = real_location;
    // accelerationLast = real_acceleration;

}


void MotionPlanner::TrajectoryTracker::CalcSoftGoal(int32_t _goalPosition,int32_t _goalVelocity, int32_t _goalAcc)
{
    //
    // if (_goalVelocity != recordVelocity || _goalPosition != recordPosition)
    // {
    //     updateTime = 0;
    //     recordPosition = _goalPosition;
    //     recordVelocity = _goalVelocity;
    //
    //     velocityNow = velocityLast;
    //     positionNow = positionLast;
    //     accelerationNow = accelerationLast;
    //
    //     velocityLast = _goalVelocity;
    //     positionLast = _goalPosition;
    //     accelerationLast = _goalAcc;
    //     overtimeFlag = false;
    // } else
    // {
    //     if (updateTime >= (updateTimeout * 1000))//每两个点之间要用200ms走完，否则就是超时
    //         overtimeFlag = true;
    //     else
    //         updateTime += context->CONTROL_PERIOD;
    // }
    //
    // if (overtimeFlag)
    // {
    //     if (velocityNow == 0)
    //     {
    //         dynamicVelocityAccRemainder = 0;
    //     } else if (velocityNow > 0)
    //     {
    //         CalcVelocityIntegral(-velocityDownAcc);
    //         if (velocityNow <= 0)
    //         {
    //             dynamicVelocityAccRemainder = 0;
    //             velocityNow = 0;
    //         }
    //     } else
    //     {
    //         CalcVelocityIntegral(velocityDownAcc);
    //         if (velocityNow >= 0)
    //         {
    //             dynamicVelocityAccRemainder = 0;
    //             velocityNow = 0;
    //         }
    //     }
    // } else
    // {
    //     CalcVelocityIntegral((int32_t)accelerationNow);
    // }
    //
    // CalcPositionIntegral(velocityNow);
    //
    // goPosition = positionNow;
    // goVelocity = velocityNow;
}


void MotionPlanner::TrajectoryTracker::CalcVelocityIntegral(int32_t value)
{
    // dynamicVelocityAccRemainder += value; // sum up last remainder
    // velocityNow += dynamicVelocityAccRemainder / context->CONTROL_FREQUENCY;
    // dynamicVelocityAccRemainder = dynamicVelocityAccRemainder % context->CONTROL_FREQUENCY; // calc remainder
}


void MotionPlanner::TrajectoryTracker::CalcPositionIntegral(int32_t value)
{
//     velovityNowRemainder += value;
//     positionNow += velovityNowRemainder /context->CONTROL_FREQUENCY;
//     velovityNowRemainder = velovityNowRemainder % context->CONTROL_FREQUENCY;
}


void MotionPlanner::TrajectoryTracker::Init(int32_t _updateTimeout)
{
    // //SetSlowDownVelocityAcc(context->config->ratedVelocityAcc / 10);
    // SetSlowDownVelocityAcc(context->config->ratedVelocityAcc*8);
    // updateTimeout = _updateTimeout;
    // // speedLockingBrake = context->config->ratedVelocityAcc / 1000;
}

void MotionPlanner::PositionTrajectoryTracker::Init(int32_t _updateTimeout)
{

    SetSlowDownVelocityAcc(context->config->ratedVelocityAcc*8);
    updateTimeout = _updateTimeout;
}
void MotionPlanner::PositionTrajectoryTracker::SetSlowDownVelocityAcc(int32_t value)
{
    velocityDownAcc = value;

}

void MotionPlanner::PositionTrajectoryTracker::NewTask(int32_t real_location, int32_t real_speed, int64_t real_acceleration)
{
    updateTime = 0;
    positionNow = real_location;
    positionLastGoal = real_location;
    recordPosition = real_location;
    velocityNow = real_speed;
    // velocityLast = real_speed;
    positionAverage = 0;
    dynamicVelocityAccRemainder = 0;
    velovityNowRemainder = 0;
    overtimeFlag = false;
}


void MotionPlanner::PositionTrajectoryTracker::CalcSoftGoal(int32_t _goalPosition,int32_t _goalVelocity, int32_t _goalAcc)
{

    if (_goalPosition != recordPosition)
    {
        updateTime = 0;
        recordPosition = _goalPosition;
        accelerationNow = ((_goalPosition-positionNow)*400-velocityNow)*400;
        if (accelerationNow>40*256*200) accelerationNow = 40*256*200;
        if (accelerationNow<-40*256*200) accelerationNow = -40*256*200;
        positionNow = positionLastGoal ;
        positionLastGoal = _goalPosition;



        overtimeFlag = false;
    } else
    {
        if (updateTime >= (updateTimeout * 1000))//每两个点之间要用200ms走完，否则就是超时
            overtimeFlag = true;
        else
            updateTime += context->CONTROL_PERIOD;
    }

    if (overtimeFlag)
    {
        if (velocityNow == 0)
        {
            dynamicVelocityAccRemainder = 0;
        } else if (velocityNow > 0)
        {
            CalcVelocityIntegral(-velocityDownAcc);
            if (velocityNow <= 0)
            {
                dynamicVelocityAccRemainder = 0;
                velocityNow = 0;
            }
        } else
        {
            CalcVelocityIntegral(velocityDownAcc);
            if (velocityNow >= 0)
            {
                dynamicVelocityAccRemainder = 0;
                velocityNow = 0;
            }
        }
        accelerationNow = 0;
    } else
    {
        CalcVelocityIntegral(accelerationNow);
    }

    CalcPositionIntegral(velocityNow);

    goPosition = positionNow;
    goVelocity = velocityNow;
    // if(overtimeFlag)
    // {
    //     goVelocity = 0;
    //     goPosition = _goalPosition;
    // }
}
void MotionPlanner::PositionTrajectoryTracker::CalcVelocityIntegral(int32_t value)
{
    dynamicVelocityAccRemainder += value; // sum up last remainder
    velocityNow += dynamicVelocityAccRemainder / context->CONTROL_FREQUENCY;
    dynamicVelocityAccRemainder = dynamicVelocityAccRemainder % context->CONTROL_FREQUENCY; // calc remainder
}


void MotionPlanner::PositionTrajectoryTracker::CalcPositionIntegral(int32_t value)
{
    velovityNowRemainder += value;
    positionNow += velovityNowRemainder /context->CONTROL_FREQUENCY;
    velovityNowRemainder = velovityNowRemainder % context->CONTROL_FREQUENCY;
}
void MotionPlanner::AttachConfig(MotionPlanner::Config_t* _config)
{
    config = _config;

    currentTracker.Init();
    velocityTracker.Init();
    positionTracker.Init();
    positionInterpolator.Init();
    trajectoryTracker.Init(30);
    positionTrajectoryTracker.Init(30);
}
