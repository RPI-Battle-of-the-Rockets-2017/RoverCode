#ifndef ROVER_IMU_TASKSCHEDULER_H
#define ROVER_IMU_TASKSCHEDULER_H


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
            Task(void (*func_)(void*), void * context_) : func(func_), context(context_){}
            Task(void (*func_)(void*)) : func(func_), context(NULL){}
            void runTask(){
                (*func)(context);
            }
            void addTask(Task* next){
                nextTask = next;
            }
            uint32_t time = 0;
            Task* nextTask = NULL;
        };
    private:
        Task* highPriority;
        Task* medPriority;
        Task* lowPriority;
        uint32_t earliestTime;
        //Find out what units
        int timeBuffer = 100;
    public:
        typedef enum {
            HIGH_PRIORITY,
            MEDIUM_PRIORITY,
            LOW_PRIORITY
        } Priority;

        TaskScheduler(int8_t timer = -1);
        //boolean addTimerTask();
        boolean addTask(TaskScheduler::Task* task, Priority priority, uint32_t timeToRun = 0);
        void runTasks(int timeLimit);
};

}

#endif
