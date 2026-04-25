#ifndef _BAYWATCHER_MENU_H_
#define _BAYWATCHER_MENU_H_

#include <stdint.h>

// 按键操作枚举
enum class KeyOp { Up, Down, OK, Cancel, Back, None };

// enum class MenuPage {
//     PAGE_MAIN = 0,
//     PAGE_MONITOR,
//     // PAGE_MODE,
//     PAGE_PID_SELECT,
//     // PAGE_PID_ANGLE,
//     // PAGE_PID_YAWSPEED,
//     PAGE_FUZZY,
//     PAGE_PID_LEFT,
//     PAGE_PID_RIGHT
// };

// class BayWatcher_Menu {
// public:
//     // 单例模式：整个程序共享这一个菜单对象
//     static BayWatcher_Menu& getInstance() {
//         static BayWatcher_Menu instance;
//         return instance;
//     }

//     void init();
//     void show();
//     void refresh_menu(KeyOp key);
//     void PeriodicTask();
// private:
//     BayWatcher_Menu()=default; // 私有构造
//     MenuPage page;
//     uint8_t  highlight_num;
//     bool     is_editing;
//     float* edit_ptr;
//     float    edit_step;
//     float    data_max;
//     float    data_min;

//     void limit_val(float *val, float min, float max);
//     void bindEditPointer();
//     void drawItem(uint8_t row, const char* name, bool selected);
//     void drawValue(uint8_t col, uint8_t row, float val, bool editing);
// };
// #endif

class BayWatcher_Menu {
public:
    static BayWatcher_Menu& getInstance() {
        static BayWatcher_Menu instance;
        return instance;
    }

    void init();
    void show();
    void refresh_menu(KeyOp key);

    // 状态变量（完全模仿你的 NEUQ_Menu 结构）
    uint8_t menu_num;            // 当前处于哪个页面
    uint8_t highlight_num;       // 当前高亮的是哪一行
    uint8_t write_ena;           // 写使能：1=正在调参，0=只是浏览
    float* write_pointer;       // 指向当前正在修改的变量
    bool*  write_pointer_b;     // 指向当前正在修改的布尔变量
    float   write_step;          // 加减的步长
    float   data_max;            // 最大值
    float   data_min;            // 最小值

private:
    BayWatcher_Menu() = default;
};

#endif