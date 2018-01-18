#ifndef ROVER_IMU_TASKSCHEDULER_H
#define ROVER_IMU_TASKSCHEDULER_H

#include "Arduino.h"
//WIP!

namespace Rover {

class TaskScheduler {
    public:
        //This task class serves to encapsulate tasks. It needs a forwarding function.
        class Task {
        private:
            void *context;
            void (*func)(void*);
        private:
            Task(void (*func_)(void*), void * context_ = NULL, uint32_t time_) : func(func_), context(context_), time(time){}
            void runTask(){
                (*func)(context);
            }
            void addTask(Task* next){
                nextTask = next;
            }
            Task* nextTask = NULL;
        };
    private:
        Task* highPriority = NULL;
        Task* medPriority = NULL;
        Task* lowPriority = NULL;
        Task* timePriority = NULL;
        uint32_t earliestTime = 0;
        //milliseconds
        int timeBuffer = 50;
    public:
        typedef enum {
            HIGH_PRIORITY,
            MEDIUM_PRIORITY,
            LOW_PRIORITY,
            TIME_PRIORITY
        } Priority;

        TaskScheduler(int8_t timer = -1);
        //boolean addTimerTask();
        void addTask(Task* task, Priority priority);
        void runTasks(int timeLimit);
};

}

#endif
