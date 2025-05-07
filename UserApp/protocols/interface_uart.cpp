#include "common_inc.h"
#include "configurations.h"

extern Motor motor;
uint32_t id, dce_kp,dce_kv,dce_ki,dce_kd;
extern EncoderCalibrator encoderCalibrator;
bool flag_conduct = false;

void OnUartCmd(uint8_t* _data, uint16_t _len)
{
    float cur, pos, vel, time;
    int ret = 0;
    int32_t pid;
    float acc_t,acc_p;

    switch (_data[0])
    {
        case 's':
            if (motor.controller->modeRunning != Motor::MODE_STOP)
                motor.controller->SetCtrlMode(Motor::MODE_STOP);
            break;
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
            flag_conduct = false;
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
        // case 't':
        //     if (motor.controller->modeRunning != Motor::MODE_COMMAND_Trajectory)
        //         motor.controller->requestMode = Motor::MODE_COMMAND_Trajectory;
        //     flag_conduct = true;
        //     break;
        case 'a':
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION_TRAJECTORY)
                motor.controller->requestMode = Motor::MODE_COMMAND_POSITION_TRAJECTORY;
            flag_conduct = true;
            break;
        case 'r':
            acc_t = motor.motionPlanner.positionTracker.GetVelocityAcc();
            acc_p = (float)motor.config.motionParams.ratedVelocityAcc/(256*200.f);
            printf("Acceleration: %.1f, %.1f\n", acc_t, acc_p);
            break;
        case 'h':
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
                motor.controller->requestMode = Motor::MODE_COMMAND_POSITION;
            motor.motionPlanner.positionTracker.SetVelocityAcc(1000*motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
            flag_conduct = true;
            break;
        case 'z':
            encoderCalibrator.isTriggered = true;
            break;
        case 'i':
            id =boardConfig.canNodeId;
            dce_kp = boardConfig.dce_kp;
            dce_kv = boardConfig.dce_kv;
            dce_ki = boardConfig.dce_ki;
            dce_kd = boardConfig.dce_kd;
            printf("CAN Node ID: %d\nkp: %d\nkv: %d\nki: %d\nkd: %d\n", id, dce_kp, dce_kv, dce_ki, dce_kd);
            break;
        case 'k':
            switch (_data[1])
            {
                case 'p':
                    ret = sscanf((char*) _data, "kp %d", &pid);
                    if (ret == 1)
                    {
                        motor.config.ctrlParams.dce.kp = pid;
                        boardConfig.dce_kp = pid;
                        boardConfig.configStatus = CONFIG_COMMIT;
                    }
                    break;
                case 'v':
                    ret = sscanf((char*) _data, "kv %d", &pid);
                    if (ret == 1)
                    {
                        motor.config.ctrlParams.dce.kv = pid;
                        boardConfig.dce_kv = pid;
                        boardConfig.configStatus = CONFIG_COMMIT;
                    }
                    break;
                case 'i':
                    ret = sscanf((char*) _data, "ki %d", &pid);
                    if (ret == 1)
                    {
                        motor.config.ctrlParams.dce.ki = pid;
                        boardConfig.dce_ki = pid;
                        boardConfig.configStatus = CONFIG_COMMIT;
                    }
                    break;
                case 'd':
                    ret = sscanf((char*) _data, "kd %d", &pid);
                    if (ret == 1)
                    {
                        motor.config.ctrlParams.dce.kd = pid;
                        boardConfig.dce_kd = pid;
                        boardConfig.configStatus = CONFIG_COMMIT;
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            printf("[error] Unknown command!\r\n");
            break;
    }
}

