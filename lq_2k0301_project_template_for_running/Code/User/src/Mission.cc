#include "Mission.h"
#include <stdio.h>

// // 实例化全局对象
// BayWatcher_Mission mission_sys;

BayWatcher_Mission::BayWatcher_Mission() : total_board(0), current_board_index(0) {
}

BayWatcher_Mission::~BayWatcher_Mission() {
}

// ================== 菜单交互接口 ==================

void BayWatcher_Mission::set_total_boards(int n) {
    if (n < 0) n = 0; // 防止人为输入负数
    total_board = n;
    
    // 极其方便的 C++ 特性：直接重新分配数组大小
    // 如果把 3 改成 5，会自动增加 2 个默认值为直行的任务
    // 如果把 5 改成 3，会自动删掉最后 2 个任务
    board_sequence.resize(total_board, HEADING_FORWARD);
    
    printf("[Mission] 目标板总数已设为: %d\n", total_board);
}

int BayWatcher_Mission::get_total_boards() const {
    return total_board;
}

void BayWatcher_Mission::set_board_direction(int index, uint8_t direction) {
    // 菜单传进来的序号是 1 到 n，内部数组是 0 到 n-1，必须做安全保护
    if (index >= 1 && index <= total_board) {
        board_sequence[index - 1] = direction;
    }
}

uint8_t BayWatcher_Mission::get_board_direction(int index) const {
    if (index >= 1 && index <= total_board) {
        return board_sequence[index - 1];
    }
    return HEADING_FORWARD; // 越界兜底
}

// ================== 赛道执行接口 ==================

uint8_t BayWatcher_Mission::get_current_turn_method() {
    if (is_mission_complete() || total_board == 0) {
        return HEADING_FORWARD; // 没录入任务或全跑完了，默认直行兜底
    }
    return board_sequence[current_board_index];
}

void BayWatcher_Mission::mark_board_passed() {
    if (!is_mission_complete()) {
        current_board_index++;
        printf("[Mission] 完成！准备迎接第 %d 个目标板\n", current_board_index + 1);
    }
}

bool BayWatcher_Mission::is_mission_complete() {
    return current_board_index >= total_board;
}

void BayWatcher_Mission::reset_mission() {
    current_board_index = 0;
    printf("[Mission] 发车准备完毕，进度归零！\n");
}

void BayWatcher_Mission::reset_total(){
    
}

int BayWatcher_Mission::get_current_board_index() const {
    // 给屏幕显示用的，为了符合人类习惯，加 1
    return current_board_index + 1; 
}