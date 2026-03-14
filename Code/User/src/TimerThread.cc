#include "TimerThread.h"

/******************************************************************
 * @brief       构造函数
 * 
 * @param       func                    传入周期控制函数
 * @param       arg                     传入回调函数的参数, 没有设置为NULL
 * @param       interval_ms             执行间隔（毫秒），默认10ms
 * 
 * @example     TimerThread thread1(xxxxx, NULL, 10);
 ******************************************************************/
TimerThread::TimerThread(CallbackFunc func, void* arg, unsigned int interval_ms)
{
    this->func = func;
    this->arg = arg;
    this->interval_ms = interval_ms;
    this->thread_id = 0;
}

TimerThread::~TimerThread(void){}

/******************************************************************
 * @brief       启动定时线程
 * 
 * @return      成功返回true，失败返回false
 * 
 * @example     thread1.start();
 * 
 * @note        线程启动后自动与主线程分离
 ******************************************************************/
bool TimerThread::start() 
{
    if (pthread_create(&thread_id, NULL, thread_entry, this) != 0) {
        return false;
    }
    
    // 设置线程为分离状态
    pthread_detach(thread_id);
    return true;
}

/******************************************************************
 * @brief       定时线程
 * 
 * @note        已经实现了定时功能,创建的任务无须套用死循环
 ******************************************************************/
void* TimerThread::thread_entry(void* param)
{
    TimerThread* self = (TimerThread*)param;
    
    struct timespec sleep_time;
    sleep_time.tv_sec = self->interval_ms / 1000;
    sleep_time.tv_nsec = (self->interval_ms % 1000) * 1000000L;
    
    while (1) {

        if (self->func) {
            self->func(self->arg);
        }

        nanosleep(&sleep_time, NULL);
    }
    
    return NULL;
}