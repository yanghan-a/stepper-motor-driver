#include "led_base.h"

void LedBase::Tick(uint32_t _timeElapseMillis, Motor::State_t _state)
{
    timer += _timeElapseMillis;

    switch (_state)
    {
        case Motor::STATE_NO_CALIB://LED1 is on, LED2 shines once
            motorEnable = false;
            heartBeatEnable = false;
            targetBlinkNum = 1;
            break;
        case Motor::STATE_RUNNING://LED1 shine twice, LED2 is on
            motorEnable = true;
            heartBeatEnable = true;
            targetBlinkNum = 0;
            break;
        case Motor::STATE_FINISH://LED1 is off, LED2 is on
            motorEnable = true;
            heartBeatEnable = false;
            targetBlinkNum = 0;
            break;
        case Motor::STATE_STOP://LED1 is on, LED2 is on
            motorEnable = false;
            heartBeatEnable = false;
            targetBlinkNum = 0;
            break;
        case Motor::STATE_OVERLOAD://LED1 is off, LED2 shines three times
            motorEnable = true;
            heartBeatEnable = false;
            targetBlinkNum = 3;
            break;
        case Motor::STATE_STALL://LED1 is on, LED2 shines twice
            motorEnable = false;
            heartBeatEnable = false;
            targetBlinkNum = 2;
            break;
    }

    if (motorEnable)
        if (heartBeatEnable)
        {
            switch (heartBeatPhase)
            {
                case 1:
                    if (timer - timerHeartBeat > 200)
                    {
                        SetLedState(0, false);
                        timerHeartBeat = timer;
                        heartBeatPhase = 2;
                    }
                    break;
                case 2:
                    if (timer - timerHeartBeat > 200)
                    {
                        SetLedState(0, true);
                        timerHeartBeat = timer;
                        heartBeatPhase = 3;
                    }
                    break;
                case 3:
                    if (timer - timerHeartBeat > 200)
                    {
                        SetLedState(0, false);
                        timerHeartBeat = timer;
                        heartBeatPhase = 4;
                    }
                    break;
                case 4:
                    if (timer - timerHeartBeat > 700)
                    {
                        SetLedState(0, true);
                        timerHeartBeat = timer;
                        heartBeatPhase = 1;
                    }
                    break;
            }
        } else
        {
            SetLedState(0, true);
            heartBeatPhase = 1;
        }
    else
        SetLedState(0, false);


    switch (blinkPhase)
    {
        case 1:
            if (timer - timerBlink > 200)
            {
                SetLedState(1, false);
                timerBlink = timer;
                blinkPhase = 2;
            }
            break;
        case 2:
            if (timer - timerBlink > 200)
            {
                blinkNum++;
                if (targetBlinkNum > blinkNum)
                {
                    SetLedState(1, true);
                    blinkPhase = 1;
                    timerBlink = timer;
                } else
                {
                    SetLedState(1, false);
                    blinkPhase = 3;
                    timerBlink = timer;
                }
            }
            break;
        case 3:
            if (timer - timerBlink > 1000)
            {
                blinkNum = 0;
                SetLedState(1, targetBlinkNum > 0);
                timerBlink = timer;
                blinkPhase = 1;
            }
            break;
    }

}


