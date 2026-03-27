#ifndef __BAYWATCHER_MISSION_H_
#define __BAYWATCHER_MISSION_H_

#include "Communication.h"
#include <stdint.h>
#include <vector>
class BayWatcher_Mission {
public:
    BayWatcher_Mission();
    ~BayWatcher_Mission();

    // ================== 菜单交互接口 ==================
    void set_total_boards(int n);                            // 录入目标板总数 n (total_board)
    int  get_total_boards() const;                           // 获取目标板总数
    void set_board_direction(int index, uint8_t direction);  // 设定特定序号目标板的绕行方式 (index  1 ~ n)
    uint8_t get_board_direction(int index) const;            // 获取特定序号目标板的绕行方式 


    // ================== 赛道执行接口  ==================
    
    // 遇到红板时，获取当前应该执行的绕行方向
    uint8_t get_current_turn_method();

    // 动作执行完毕后调用，进度推入下一个板子
    void mark_board_passed();

    // 检查任务是否已经全部跑完
    bool is_mission_complete();

    // 重新发车时调用，让进度回到第 1 个板子
    void reset_mission();
    void reset_total();

    // 获取当前正在面对第几个板子 (返回 1 到 n，用于发车后的实时调试显示)
    int get_current_board_index() const;
    int total_board;                     // 总计目标板数量

private:
    std::vector<uint8_t> board_sequence; // 存储每个板子的绕行方向
    int current_board_index;             // 当前跑到第几个板子了 (内部从 0 开始计算)
};

// 声明全局对象
// extern BayWatcher_Mission mission_sys;
#endif // !__BAYWATCHER_MISSION_H_