#ifndef _BAYWATCHER_TIMERTHREAD_H_
#define _BAYWATCHER_TIMERTHREAD_H_

#include "main.hpp"

class TimerThread {
public:
/******************************************************************
 * @brief   线程回调函数原型
 ******************************************************************/
typedef void (*CallbackFunc)(void*);
    
/******************************************************************
 * @brief       构造函数
 * 
 * @param       func                    传入周期控制函数
 * @param       arg                     传入回调函数的参数, 没有设置为NULL
 * @param       interval_ms             执行间隔（毫秒），默认10ms
 * 
 * @example     TimerThread thread1(xxxxx, NULL, 10);
 ******************************************************************/
    TimerThread(CallbackFunc func, void* arg = NULL, unsigned int interval_ms = 10);
    ~TimerThread(void);
    
/******************************************************************
 * @brief       启动定时线程
 * 
 * @return      成功返回true，失败返回false
 * 
 * @example     thread1.start();
 * 
 * @note        线程启动后自动与主线程分离
 ******************************************************************/
    bool start();
    
private:
    static void* thread_entry(void* param);
    pthread_t thread_id;
    CallbackFunc func;
    void* arg;
    unsigned int interval_ms;
    
};

#endif 