#ifndef ROVER_IMU_TASKSCHEDULER_H
#define ROVER_IMU_TASKSCHEDULER_H


//WIP!

namespace Rover {

class TaskScheduler {
    private:
        void** highPriority;
        void** medPriority;
        void** lowPriority;
    public:
        //This task class serves to encapsulate tasks and
        // forwards tasks to member functions. It's not needed
        // if the function is in the global namespace.
        template <typename Context_t> class Task {
        private:
            Context_t& context;
            void (*func)();
        private:
            Task(Context_t& context_, void (*func_)()) : context(context_), func(func){}
            void runTask(){
                (context.*func)();
            }
        };
        TaskScheduler(int8_t timer = -1);
        boolean addTimerTask();
        boolean addTask(void (*func)(), );
        boolean addTask(Task& task, );
        void runTasks(int timeLimit);


};

}

#endif
