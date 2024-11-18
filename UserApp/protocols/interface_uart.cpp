#include "common_inc.h"
#include "configurations.h"

float buffer_pos[400];
float buffer_vel[400];
int buffer_len = 0;
extern Motor motor;

void OnUartCmd(uint8_t* _data, uint16_t _len)
{
    float cur, pos, vel, time;
    int ret = 0;

    switch (_data[0])
    {
        case 'c':
            ret = sscanf((char*) _data, "c %f", &cur);
            if (ret < 1)
            {
                printf("[error] Command format error!\r\n");
            } else if (ret == 1)
            {
                if (motor.controller->modeRunning != Motor::MODE_COMMAND_CURRENT)
                    motor.controller->SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
                motor.controller->SetCurrentSetPoint((int32_t) (cur * 1000));
            }
            break;
        case 'v':
            ret = sscanf((char*) _data, "v %f", &vel);
            if (ret < 1)
            {
                printf("[error] Command format error!\r\n");
            } else if (ret == 1)
            {
                if (motor.controller->modeRunning != Motor::MODE_COMMAND_VELOCITY)
                {
                    motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                    motor.controller->SetCtrlMode(Motor::MODE_COMMAND_VELOCITY);
                }
                motor.controller->SetVelocitySetPoint(
                    (int32_t) (vel * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            }
            break;
        case 'p':
            ret = sscanf((char*) _data, "p %f", &pos);
            if (ret < 1)
            {
                printf("[error] Command format error!\r\n");
            } else if (ret == 1)
            {
                if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
                    motor.controller->requestMode = Motor::MODE_COMMAND_POSITION;

                motor.controller->SetPositionSetPoint(
                    (int32_t) (pos * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            }
            break;
        case 't':
            ret = sscanf((char*) _data, "t %f %f", &pos, &vel);
            if (ret < 2)
            {
                printf("[error] Command format error!\r\n");

            } else if (ret == 2)
            {
                // printf("t %.4f %.4f\r\n", pos, vel);
                // if (motor.controller->modeRunning != Motor::MODE_COMMAND_Trajectory)
                //     motor.controller->requestMode = Motor::MODE_COMMAND_Trajectory;
                //
                // motor.controller->AddTrajectorySetPoint((int32_t) (pos * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS),
                //        (int32_t) (vel* (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
                buffer_pos[buffer_len] = pos;
                buffer_vel[buffer_len] = vel;
                buffer_len++;
                if (buffer_len == sizeof(buffer_pos) / sizeof(buffer_pos[0]))
                {
                    buffer_len = 0;
                }
                break;
            }
        // case 'ratchet wheel':
        //     ret = sscanf((char*) _data, "Ratchet wheel %f %f %f %f", &cur, &pos, &vel, &time);
        //     if (ret < 4)
        //     {
        //         printf("[error] Command format error!\r\n");
        //     } else if (ret == 4)
        //     {
        //         if (motor.controller->modeRunning != Motor::MODE_COMMAND_CURRENT)
        //             motor.controller->SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
        //         motor.controller->SetCurrentSetPoint((int32_t) (cur * 1000));
        //         motor.controller->SetPositionSetPoint(
        //             (int32_t) (pos * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
        //         motor.controller->SetVelocitySetPoint(
        //             (int32_t) (vel * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
        //         motor.controller->SetTimeSetPoint((int32_t) (time * 1000));
        //     }
        //     break;
        // default:
        //     printf("[error] Unknown command!\r\n");
        //     break;
    }
}

