#include <cmath>
#include <string.h>
#include "math.h"
#include "common_inc.h"
#include "configurations.h"
#include "Platform/Utils/st_hardware.h"
#include <tim.h>

#include "usart.h"

/* Component Definitions -----------------------------------------------------*/
BoardConfig_t boardConfig;
Motor motor;
TB67H450 tb67H450;
MT6816 mt6816;
EncoderCalibrator encoderCalibrator(&motor);
Button button1(1, 1000), button2(2, 3000);
void OnButton1Event(Button::Event _event);
void OnButton2Event(Button::Event _event);
Led statusLed;

int flag_velocity = 0;
int flag_mt6816 = 0;
/* Main Entry ----------------------------------------------------------------*/
void Main()
{
    uint64_t serialNum = GetSerialNumber();
    uint16_t defaultNodeID = 0;
    //ID0 is PA8 -->  Switch 4
    //ID1 is PA9 -->  Switch 3
    //ID2 is PA10 --> Switch 2
    uint16_t nodeID = !HAL_GPIO_ReadPin(GPIOA, ID0_Pin);
    nodeID |= !HAL_GPIO_ReadPin(GPIOA, ID1_Pin) << 1;
    nodeID |= !HAL_GPIO_ReadPin(GPIOA, ID2_Pin) << 2;
    defaultNodeID = nodeID;
    // Change below to fit your situation
    // switch (serialNum)
    // {
    //     case 431466563640: //J1
    //         defaultNodeID = 1;
    //         break;
    //     case 384624576568: //J2
    //         defaultNodeID = 2;
    //         break;
    //     case 384290670648: //J3
    //         defaultNodeID = 3;
    //         break;
    //     case 431531051064: //J4
    //         defaultNodeID = 4;
    //         break;
    //     case 431466760248: //J5
    //         defaultNodeID = 5;
    //         break;
    //     case 431484848184: //J6
    //         defaultNodeID = 6;
    //         break;
    //     default:
    //         break;
    // }


    /*---------- Apply EEPROM Settings ----------*/
    // Setting priority is EEPROM > Motor.h
    EEPROM eeprom;
    // eeprom.get(0, boardConfig);
    if (boardConfig.configStatus != CONFIG_OK) // use default settings
    {
        boardConfig = BoardConfig_t{
            .configStatus = CONFIG_OK,
            .canNodeId = defaultNodeID,
            .encoderHomeOffset = 0,
            .defaultMode = Motor::MODE_COMMAND_POSITION,
            .currentLimit = 1 * 2000,    // mA
            .velocityLimit = 30 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, // r/s
            .velocityAcc = 100 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS,   // r/s^2
            .calibrationCurrent=2000,// vector of currents for calibration
            .dce_kp = 2000,// 200,
            .dce_kv = 120,//80,
            .dce_ki = 200,//300,
            .dce_kd = 300,//250,
            .enableMotorOnBoot=false,// enable motor to the defaultMode
            .enableStallProtect=false,
            .enableTempWatch=true,
        };
        // eeprom.put(0, boardConfig);
    }
    boardConfig.enableTempWatch=true;
    motor.config.motionParams.encoderHomeOffset = boardConfig.encoderHomeOffset;
    motor.config.motionParams.ratedCurrent = boardConfig.currentLimit;
    motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
    motor.config.motionParams.ratedVelocityAcc = boardConfig.velocityAcc;
    motor.motionPlanner.velocityTracker.SetVelocityAcc(boardConfig.velocityAcc);
    motor.motionPlanner.positionTracker.SetVelocityAcc(boardConfig.velocityAcc);
    motor.config.motionParams.caliCurrent = boardConfig.calibrationCurrent;

    motor.config.ctrlParams.dce.kp = boardConfig.dce_kp;
    motor.config.ctrlParams.dce.kv = boardConfig.dce_kv;
    motor.config.ctrlParams.dce.ki = boardConfig.dce_ki;
    motor.config.ctrlParams.dce.kd = boardConfig.dce_kd;
    motor.config.ctrlParams.stallProtectSwitch = boardConfig.enableStallProtect;


    /*---------------- Init Motor ----------------*/
    motor.AttachDriver(&tb67H450);
    motor.AttachEncoder(&mt6816);
    motor.controller->Init();
    motor.driver->Init();
    motor.encoder->Init();


    /*------------- Init peripherals -------------*/
    button1.SetOnEventListener(OnButton1Event);
    button2.SetOnEventListener(OnButton2Event);


    /*------- Start Close-Loop Control Tick ------*/
    HAL_Delay(100);
    HAL_TIM_Base_Start_IT(&htim3);  // 1000Hz
    HAL_TIM_Base_Start_IT(&htim1);  // 100Hz
    HAL_TIM_Base_Start_IT(&htim4);  // 20kHz

    if (button1.IsPressed() && button2.IsPressed())
        encoderCalibrator.isTriggered = true;
    //encoderCalibrator.isTriggered = true;

    // motor.controller->SetCtrlMode(Motor::MODE_PWM_VELOCITY);
    // motor.controller->SetVelocitySetPoint(20*motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);

    // motor.controller->SetCtrlMode(Motor::MODE_PWM_POSITION);
    // motor.controller->SetPositionSetPoint(100*motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);

    for (;;)
    {
        encoderCalibrator.TickMainLoop();


        if (boardConfig.configStatus == CONFIG_COMMIT)
        {
            boardConfig.configStatus = CONFIG_OK;
            eeprom.put(0, boardConfig);
        } else if (boardConfig.configStatus == CONFIG_RESTORE)
        {
            eeprom.put(0, boardConfig);
            HAL_NVIC_SystemReset();
        }
    }
}
uint8_t parameters[24];
float position, velocity, acceleration, foc_current;
float goal_position, goal_velocity;
int32_t point;
float foc_current_last;
/* Event Callbacks -----------------------------------------------------------*/
extern "C" void Tim3Callback20Hz()//1000Hz
{

    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    // printf("%d,%d,%d,%d\n", motor.controller->config->dce.vError,motor.controller->config->dce.real_vError, motor.controller->config->dce.pError,motor.controller->config->dce.real_pError);
    // printf("%.4f,%.4f,%.4f,%.4f\n",motor.controller->GetVelocity(), motor.controller->GetPosition(), motor.controller->GetFocCurrent(),motor.controller->GetAcceleration());
    // position = motor.controller->GetPosition();
    // velocity = motor.controller->GetVelocity();
    // acceleration = motor.controller->GetAcceleration();
    // foc_current = motor.controller->GetFocCurrent()*10.f;
    // memcpy(parameters + 0 * sizeof(float), &position, sizeof(float));
    // memcpy(parameters + 1 * sizeof(float), &velocity, sizeof(float));
    // memcpy(parameters + 2 * sizeof(float), &acceleration, sizeof(float));
    // memcpy(parameters + 3 * sizeof(float), &foc_current, sizeof(float));
    // parameters[16]=0x00;
    // parameters[17]=0x00;
    // parameters[18]=0x80;
    // parameters[19]=0x7f;
    position = motor.controller->GetPosition();
    velocity = motor.controller->GetVelocity();
    goal_position = motor.controller->getGoalPosition();
    goal_velocity = motor.controller->getGoalVelocity();
    foc_current = motor.controller->GetFocCurrent()*0.1f+foc_current_last*0.9f;
    memcpy(parameters + 0 * sizeof(float), &position, sizeof(float));
    memcpy(parameters + 1 * sizeof(float), &velocity, sizeof(float));
    memcpy(parameters + 2 * sizeof(float), &goal_position, sizeof(float));
    memcpy(parameters + 3 * sizeof(float), &goal_velocity, sizeof(float));
    memcpy(parameters + 4 * sizeof(float), &foc_current, sizeof(float));
    parameters[20]=0x00;
    parameters[21]=0x00;
    parameters[22]=0x80;
    parameters[23]=0x7f;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)parameters, 24);

    foc_current_last = foc_current;
}
/* Event Callbacks -----------------------------------------------------------*/
uint32_t count;
extern float buffer_pos[400];
extern float buffer_vel[400];
extern int buffer_len;
extern "C" void Tim1Callback100Hz()
{
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

    button1.Tick(10);
    button2.Tick(10);
    statusLed.Tick(10, motor.controller->state);

    point ++;
    if(buffer_len>0)
    {
        // float pos = 10*cos((float)buffer_pos[point%256]/400.f*2*3.14159265358979323846f)-10;
        // float vel = -10*sin((float)buffer_vel[point%256]/400.f*2*3.14159265358979323846f)*2*3.14159265358979323846f/4.f;
        float pos = buffer_pos[point%((sizeof(buffer_pos) / sizeof(buffer_pos[0])))];
        float vel = buffer_vel[point%((sizeof(buffer_pos) / sizeof(buffer_pos[0])))];
        motor.controller->AddTrajectorySetPoint((int32_t) (pos * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS),
                           (int32_t) (vel* (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
    }


    count++;
    // if (boardConfig.enableTempWatch)
    // {
    //     count ++;
    //     if ( count >= 100)
    //     {
    //         boardConfig.motor_temperature = AdcGetChipTemperature();
    //         count = 0;
    //     }
    // }
}


extern "C" void Tim4Callback20kHz()
{
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);

    if (encoderCalibrator.isTriggered)
        encoderCalibrator.Tick20kHz();
    else
        motor.Tick20kHz();
}


void OnButton1Event(Button::Event _event)
{
    switch (_event)
    {
        case ButtonBase::UP:
            break;
        case ButtonBase::DOWN:
            break;
        case ButtonBase::LONG_PRESS:
            encoderCalibrator.isTriggered = true;

            // HAL_NVIC_SystemReset();
            break;
        case ButtonBase::CLICK://CLICK is used to stop and store current mode
            printf("KEY1\r\n");
            if (motor.controller->modeRunning != Motor::MODE_STOP)
            {
                boardConfig.defaultMode = motor.controller->modeRunning;
                motor.controller->requestMode = Motor::MODE_STOP;
            } else
            {
                motor.controller->requestMode = static_cast<Motor::Mode_t>(boardConfig.defaultMode);
            }
            point = 0;
            break;
    }
}


void OnButton2Event(Button::Event _event)
{
    switch (_event)
    {
        case ButtonBase::UP:
            break;
        case ButtonBase::DOWN:
            break;
        case ButtonBase::LONG_PRESS:
            switch (motor.controller->modeRunning)
            {
                case Motor::MODE_COMMAND_CURRENT:
                case Motor::MODE_PWM_CURRENT:
                    motor.controller->SetCurrentSetPoint(0);
                    break;
                case Motor::MODE_COMMAND_VELOCITY:
                case Motor::MODE_PWM_VELOCITY:
                    motor.controller->SetVelocitySetPoint(0);
                    break;
                case Motor::MODE_COMMAND_POSITION:
                case Motor::MODE_PWM_POSITION:
                    motor.controller->SetPositionSetPoint(0);
                    break;
                case Motor::MODE_COMMAND_Trajectory:
                case Motor::MODE_STEP_DIR:
                case Motor::MODE_STOP:
                    break;
            }
            break;
        case ButtonBase::CLICK:
            printf("KEY2\r\n");

            motor.controller->ClearStallFlag();
            break;
    }
}