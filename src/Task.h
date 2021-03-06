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

#pragma once

// if you need finer timing resolution than a millisecond
// then enable the below, but this will limit the max interval 
// to just over 70 minutes
// #define TASK_MICRO_RESOLUTION

#if defined TASK_MICRO_RESOLUTION
uint32_t badTaskTimeValue(void)
    __attribute__((error("constant is too large for 'TaskTime' type: max is 4,294,967 milliseconds")));
#define TaskTimePerMs 1000
#define TaskTimeAccuracy 10
#define GetTaskTime micros
#define TaskTimeValidate(ms) ((ms > (uint32_t(-1) / (uint32_t)TaskTimePerMs)) ? badTaskTimeValue() : ms)
#define MsToTaskTime(ms) (TaskTimeValidate(ms) * (uint32_t)TaskTimePerMs)
#define UsToTaskTime(us) ((uint32_t)us)
#define TaskTimeToMs(t) (t / (uint32_t)TaskTimePerMs)
#define TaskTimeToUs(t) ((uint32_t)t)
#else
//uint32_t badTaskTimeValue(void)
//    __attribute__((error("constant is too large for 'microseconds' conversion: max is 4,294,967 milliseconds")));
#define badTaskTimeValue() 1
#define TaskTimePerMs 1
#define TaskTimeAccuracy 1
#define GetTaskTime millis
#define MsToTaskTime(ms) ((uint32_t)ms)
#define UsToTaskTime(ms) (ms / (uint32_t)1000)
#define TaskTimeToMs(t) ((uint32_t)t)
#define TaskTimeValidate(t) ((t > (uint32_t(-1) / (uint32_t)1000)) ? badTaskTimeValue() : t)
#define TaskTimeToUs(t) (TaskTimeValidate(t) * (uint32_t)1000)
#endif

enum TaskState
{
    TaskState_Stopped,
    TaskState_Running,
    TaskState_Stopping,
    TaskState_Suspended,
    TaskState_Ready
};

class Task
{
public:
    Task(uint32_t timeInterval) :
            _remainingTime(0),
            _timeInterval(timeInterval),
            _pNext(NULL),
            _taskState(TaskState_Stopped),
            _updateTimeReached(false),
            _dirtyRemainingTime(false)
    {

    }

    void setTimeInterval(uint32_t timeInterval)
    {
        _timeInterval = timeInterval;
        if (_taskState == TaskState_Running
                || _taskState == TaskState_Ready
                || _taskState == TaskState_Suspended)
        {
            _remainingTime = timeInterval;
            _dirtyRemainingTime = false;
        }
    }

    uint32_t getTimeInterval()
    {
        return _timeInterval;
    }

    TaskState getTaskState()
    {
        return _taskState;
    }

protected:
    virtual bool OnStart() { return true; };
    virtual void OnStop() {};
    virtual bool OnSuspend() { return true; };
    virtual bool OnResume() { return true; };
    virtual void OnUpdate(uint32_t deltaTime) {};

    uint32_t _remainingTime;
    uint32_t _timeInterval;

    void Suspend()
    {
        if (_taskState == TaskState_Suspended
                || _taskState == TaskState_Stopped)
        {
            // a stopped task cannot suspend itself
            // use TaskManager::SuspendTask instead
            return;
        }

        if (OnSuspend())
        {
            _taskState = TaskState_Suspended;
        }
        else
        {
            _taskState = TaskState_Stopping;
        }
    }
    void Resume()
    {
        if (_taskState == TaskState_Suspended)
        {
            if (OnResume())
            {
                _taskState = TaskState_Ready;
            }
            else
            {
                _taskState = TaskState_Stopping;
            }
        }
    }
    void setRemainingTime(uint32_t remainingTime)
    {
        _remainingTime = remainingTime;
        _dirtyRemainingTime = false;
    }
private:
    friend class TaskManager;
    Task* _pNext; // next task in list
    TaskState _taskState;
    bool _updateTimeReached;
    bool _dirtyRemainingTime;

    void Start()
    {
        _remainingTime = _timeInterval;
        _dirtyRemainingTime = false;
        _updateTimeReached = false;

        if (OnStart())
        {
            _taskState = TaskState_Running;
        }
        else
        {
            _taskState = TaskState_Stopping;
        }
    }
    void Stop()
    {
        if (_taskState == TaskState_Running
                || _taskState == TaskState_Ready
                || _taskState == TaskState_Suspended)
        {
            OnStop();
            _taskState = TaskState_Stopping;
        }
    }
};

#include "MessageTask.h"
#include "FunctionTask.h"
#include "TaskMacros.h"
#include "TaskManager.h"
