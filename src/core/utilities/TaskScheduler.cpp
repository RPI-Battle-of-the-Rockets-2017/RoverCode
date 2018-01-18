#include "Arduino.h"

#include "TaskScheduler.h"

namespace Rover {

//Not gonna make it timer compatible for now
TaskScheduler::TaskScheduler(int8_t timer){
}

void TaskScheduler::addTask(Task* task, Priority priority){
    if (task->time > 0){
        earliestTime = (task->time<earliestTime) ? task->time : earliestTime;
    }
    switch(priority){
      case HIGH_PRIORITY:
        task->nextTask = highPriority;
        highPriority = task;
      case MEDIUM_PRIORITY:
        task->nextTask = mediumPriority;
        mediumPriority = task;
      case LOW_PRIORITY:
        task->nextTask = lowPriority;
        lowPriority = task;
      case TIME_PRIORITY:
        task->nextTask = timePriority;
        timePriority = task;
    }
}

void TaskScheduler::runTasks(int timeLimit){
    while (millis() < timeLimit-timeBuffer){
        if (earliestTime < millis() && earliestTime != 0){
            Task* taskToRun = timePriority;
            Task* lastTask = timePriority;
            uint32_t newTime = 0;
            while (taskToRun->time != earliestTime && taskToRun != NULL) {
                if (newTime == 0 || taskToRun->time < newTime)
                    newTime = taskToRun->time;
                lastTask = taskToRun;
                taskToRun = taskToRun->nextTask;
            }
            taskToRun->runTask();
            lastTask->nextTask = taskToRun->nextTask;
            taskToRun = taskToRun->nextTask;
            while (taskToRun->time != earliestTime && taskToRun != NULL) {
                if (newTime == 0 || taskToRun->time < newTime)
                    newTime = taskToRun->time;
                lastTask = taskToRun;
                taskToRun = taskToRun->nextTask;
            }
            earliestTime = newTime;
            continue;
        }
        if (highPriority != NULL){
            highPriority->runTask();
            continue;
        }
        if (mediumPriority != NULL){
            mediumPriority->runTask();
            continue;
        }
        if (lowPriority != NULL){
            lowPriority->runTask();
            continue;
        }
    }
}

}
