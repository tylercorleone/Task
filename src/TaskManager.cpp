/*--------------------------------------------------------------------
Task is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of
the License, or (at your option) any later version.

Task is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

See GNU Lesser General Public License at <http://www.gnu.org/licenses/>.
--------------------------------------------------------------------*/

#include <Arduino.h>

#include "Task.h"
#include "TaskManager.h"

#if defined(ARDUINO_ARCH_ESP8266)
extern "C"
{
#include <user_interface.h>
}
#elif defined(__arm__)

#elif defined(ARDUINO_ARCH_AVR)
#include <avr/power.h>
#endif

#define USE_WDT

TaskManager::TaskManager() :
        _pFirstTask( NULL ),
        _pLastTask( NULL ),
        _taskListScanned(false),
        _runningTasksCount(0)
{

#if defined(ARDUINO_ARCH_AVR) && !defined(__arm__)
    // make sure the watch dog is disabled during setup
    // avoid this for Esp8266 due to it will only disable the software watchdog
    // but leave the hardware one to fire, further this would disable all the
    // built in hidden watchdog feed calls that would keep it from firing and
    // thus causing an effect of enabling the watchdog rather than disabling
    wdt_reset();
    wdt_disable();
#endif

    _lastTick = GetTaskTime();
}

void TaskManager::Setup()
{
    _lastTick = GetTaskTime();
}

void TaskManager::StartTask(Task* pTask)
{
    if (pTask->_taskState == TaskState_Running
            || pTask->_taskState == TaskState_Ready)
    {
        return;
    }

    // check if it has been stopped yet as it may be just stopping
    if (pTask->_taskState == TaskState_Stopped)
    {
        // append to the list
        if (_pFirstTask == NULL)
        {
            _pFirstTask = pTask;
            _pLastTask = pTask;
        }
        else
        {
            _pLastTask->_pNext = pTask;
            _pLastTask = pTask;
        }
    }
    pTask->Start();
}

void TaskManager::StopTask(Task* pTask)
{
    pTask->Stop();
}

void TaskManager::SuspendTask(Task* pTask)
{
    if (pTask->_taskState == TaskState_Suspended)
    {
        return;
    }

    // check if it has been stopped yet as it may be just stopping
    if (pTask->_taskState == TaskState_Stopped)
    {
        pTask->_remainingTime = pTask->_timeInterval;
        pTask->_dirtyRemainingTime = false;
        pTask->_updateTimeReached = false;
        pTask->_taskState = TaskState_Ready;

        // append to the list
        if (_pFirstTask == NULL)
        {
            _pFirstTask = pTask;
            _pLastTask = pTask;
        }
        else
        {
            _pLastTask->_pNext = pTask;
            _pLastTask = pTask;
        }
    }
    pTask->Suspend();
}


void TaskManager::ResumeTask(Task* pTask)
{
    pTask->Resume();
}

TaskState TaskManager::StatusTask(Task* pTask)
{
    return pTask->_taskState;
}

void TaskManager::Loop(uint16_t watchdogTimeOutFlag)
{
    uint32_t currentTick = GetTaskTime();
    uint32_t deltaTime = currentTick - _lastTick;
    _taskListScanned = false;

    if (deltaTime >= TaskTimeAccuracy)
    {
        _lastTick = currentTick; // update before calling process
        uint32_t nextWakeTime = ProcessTasks(deltaTime);
        _taskListScanned = true;

        // if the next task has more time available than the next
        // millisecond interupt, then sleep
        if (nextWakeTime > TaskTimePerMs)
        {
            // for idle sleep mode:
            // due to Millis() using timer interupt at 1 ms,
            // the cpu will be woke up by that every millisecond

#if defined(ARDUINO_ARCH_ESP8266)
            // the esp8266 really doesn't have an idle mode
#if defined(USE_WDT)
            // use watchdog timer for failsafe mode,
            // total task update time should be less than watchdogTimeOutFlag
            wdt_disable();
            wdt_enable(watchdogTimeOutFlag);
#endif

#elif defined(__arm__)
                // Arm support for sleep/idle not implemented yet

#elif defined(ARDUINO_ARCH_AVR)

#if defined(USE_WDT)
            // use watchdog timer for failsafe mode,
            // total task update time should be less than watchdogTimeOutFlag
            wdt_reset();
            wdt_enable(watchdogTimeOutFlag);
#endif

            // just sleep
            set_sleep_mode(SLEEP_MODE_IDLE);
            cli();
            sleep_enable();
#if defined(BODSE)
            // lower power trick
            // sleep_bod_disable() - i have seen this method called, but can't find it
            MCUCR |= _BV(BODS) | _BV(BODSE);  // turn on brown-out enable select
            MCUCR &= ~_BV(BODSE);        // this must be done within 4 clock cycles of above
#endif
            sei();
            sleep_cpu(); // will sleep in this call
            sleep_disable();
#endif // Arduino Normal
        }
#if defined(USE_WDT)
        else
        {
#if !defined(__arm__) // no arm support for watchdog
            wdt_reset(); // keep the dog happy
#endif
        }
#endif
    }
}

#if defined(ARDUINO_ARCH_ESP8266)
#define RTC_MEM_SLEEP_ADDR 65 // 64 is being overwritten right now

void TaskManager::EnterSleep(uint32_t microSeconds,
    void* state,
    uint16_t sizeofState,
    WakeMode mode)
{
    if (state != NULL && sizeofState > 0)
    {
        system_rtc_mem_write(RTC_MEM_SLEEP_ADDR, state, sizeofState);
    }
    ESP.deepSleep(microSeconds, mode);
}

bool TaskManager::RestartedFromSleep(void* state, uint16_t sizeofState)
{
    rst_info* resetInfo = ESP.getResetInfoPtr();
    bool wasSleeping = (resetInfo && REASON_DEEP_SLEEP_AWAKE == resetInfo->reason);
    if (wasSleeping)
    {
        if (state != NULL && sizeofState > 0)
        {
            system_rtc_mem_read(RTC_MEM_SLEEP_ADDR, state, sizeofState);
        }
    }
    return wasSleeping;
}

#elif defined(__arm__)
// Arm support for sleep not implemented yet


#elif defined(ARDUINO_ARCH_AVR)

void TaskManager::EnterSleep(uint8_t sleepMode)
{
#if defined(USE_WDT)
    // disable watchdog so it doesn't wake us up
    wdt_reset();
    wdt_disable();
#endif
    // prepare sleep
    set_sleep_mode(sleepMode);
    cli();
    sleep_enable();

#if defined(BODSE)
    // lower power trick
    // sleep_bod_disable() - i have seen this method called, but can't find it
    MCUCR |= _BV(BODS) | _BV(BODSE);  // turn on brown-out enable select
    MCUCR &= ~_BV(BODSE);        // this must be done within 4 clock cycles of above
#endif

    sei();
    sleep_cpu(); // will sleep in this call
    sleep_disable();

#if defined(USE_WDT)
    // enable watch dog after wake up
    wdt_reset();
    wdt_enable(WDTO_500MS);
#endif
}
#endif

uint32_t TaskManager::ProcessTasks(uint32_t deltaTime)
{
    // Update Tasks
    //
    Task* pIterate = _pFirstTask;
    while (pIterate != NULL)
    {
        if (pIterate->_taskState == TaskState_Ready)
        {
            pIterate->_taskState = TaskState_Running;
        }

        // skip any non running tasks
        if (pIterate->_taskState == TaskState_Running)
        {
            if (pIterate->_remainingTime <= deltaTime)
            {
                // calc per task delta time
                uint32_t taskDeltaTime = pIterate->_timeInterval - pIterate->_remainingTime;
                taskDeltaTime += deltaTime;

                pIterate->_updateTimeReached = true;
                pIterate->_dirtyRemainingTime = true;

                pIterate->OnUpdate(taskDeltaTime);
            }
        }

        pIterate = pIterate->_pNext;
    }
    uint32_t nextWakeTime = UpdateTasksList(deltaTime);
    return nextWakeTime;
}

uint32_t TaskManager::UpdateTasksList(uint32_t deltaTime)
{
    uint32_t nextWakeTime = ((uint32_t)-1); // MAX_UINT32
    _runningTasksCount = 0;

    Task* pIterate = _pFirstTask;
    Task* pIteratePrev = NULL;
    while (pIterate != NULL)
    {
        Task* pNext = pIterate->_pNext;
        if (pIterate->_taskState == TaskState_Stopping)
        {
            RemoveTask(pIterate, pIteratePrev, pNext);
        }
        else
        {
            // didn't remove, advance the previous pointer
            pIteratePrev = pIterate;

            if (pIterate->_taskState == TaskState_Running)
            {
                ++_runningTasksCount;

                if (pIterate->_dirtyRemainingTime)
                {
                    UpdateTaskRemainingTime(pIterate, deltaTime);
                }
                // let's all Running tasks have a dirtyRemainingTime (value)
                // before the next iteration
                pIterate->_dirtyRemainingTime = true;
                pIterate->_updateTimeReached = false;

                if (pIterate->_remainingTime < nextWakeTime)
                {
                    nextWakeTime = pIterate->_remainingTime;
                }
            }
        }
        pIterate = pNext; // iterate to the next
    }

    return nextWakeTime;
}

void TaskManager::RemoveTask(Task* pTaskToRemove, Task* pPrevious, Task* pNext)
{
    pTaskToRemove->_taskState = TaskState_Stopped;
    pTaskToRemove->_pNext = NULL;

    if (pTaskToRemove == _pFirstTask)
    {
        // first one, correct our first pointer
        _pFirstTask = pNext;
        if (pTaskToRemove == _pLastTask)
        {
            // last one, correct our last pointer
            _pLastTask = _pFirstTask;
        }
    }
    else
    {
        // all others correct the previous to remove it
        pPrevious->_pNext = pNext;
        if (pTaskToRemove == _pLastTask)
        {
            // last one, correct our last pointer
            _pLastTask = pPrevious;
        }
    }
}

uint32_t TaskManager::UpdateTaskRemainingTime(Task* pTask, uint32_t deltaTime)
{
    if (pTask->_updateTimeReached)
    {
        // add the initial time so we don't loose any remainders
        pTask->_remainingTime += pTask->_timeInterval;
        // if we are still less than delta time, things are running slow
        // so push to the next update frame
        if (pTask->_remainingTime <= deltaTime)
        {
            pTask->_remainingTime = deltaTime + TaskTimeAccuracy;
        }
    }

    return pTask->_remainingTime -= deltaTime;
}

