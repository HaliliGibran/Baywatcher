#include "main.hpp"
#include <stdio.h> 

void BayWatcher_Menu::init() {
    menu_num = 0;
    highlight_num = 1;
    write_ena = 0;
    write_pointer = NULL;
    write_step = 1.0f;
    data_max = 1000.0f;
    data_min = -1000.0f;
    
    // 初始化 TFT 驱动
    TFTSPI_dri_init(1);
    show();
}

void BayWatcher_Menu::show() {
    TFTSPI_dir_cls(u16BLACK); // 每次刷新前清黑屏
    char buf[32];             // 用于格式化浮点数的缓冲
    
    switch(menu_num) {
        case 0: // ================= 主菜单 =================
            TFTSPI_dir_P8X16Str(0, 0, "=== Main Menu ===", u16GREEN, u16BLACK);
            
            switch(highlight_num) {
                case 1: TFTSPI_dir_P8X16Str(0, 1, "> 1.Monitor", u16YELLOW, u16BLACK); break;
                case 2: TFTSPI_dir_P8X16Str(0, 2, "> 2.PID Select", u16YELLOW, u16BLACK); break;
                case 3: 
                    if(PID.is_running) TFTSPI_dir_P8X16Str(0, 3, "> 3.Stop Car", u16YELLOW, u16BLACK);
                    else               TFTSPI_dir_P8X16Str(0, 3, "> 3.Start Car", u16YELLOW, u16BLACK);
                    break;
            }
            
            if(highlight_num != 1) TFTSPI_dir_P8X16Str(0, 1, "  1.Monitor", u16WHITE, u16BLACK);
            if(highlight_num != 2) TFTSPI_dir_P8X16Str(0, 2, "  2.PID Select", u16WHITE, u16BLACK);
            if(highlight_num != 3) {
                if(PID.is_running) TFTSPI_dir_P8X16Str(0, 3, "  3.Stop Car", u16WHITE, u16BLACK);
                else               TFTSPI_dir_P8X16Str(0, 3, "  3.Start Car", u16WHITE, u16BLACK);
            }
            break;

        case 1: // ================= 监控页 =================
            TFTSPI_dir_P8X16Str(0, 0, "=== Monitor ===", u16GREEN, u16BLACK);
            if(write_ena) TFTSPI_dir_P8X16Str(14, 0, "[E]", u16RED, u16BLACK);

            sprintf(buf, "%sTgtSpd:%.1f", highlight_num == 1 ? "> ":"  ", PID.base_target_speed);
            TFTSPI_dir_P8X16Str(0, 1, buf, (highlight_num == 1) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);
            sprintf(buf, "  SL:%.1f SR:%.1f", vL, vR);
            TFTSPI_dir_P8X16Str(0, 2, buf, u16WHITE, u16BLACK);
            sprintf(buf, "  PL:%d PR:%d", PID.pwm_out_L, PID.pwm_out_R);
            TFTSPI_dir_P8X16Str(0, 3, buf, u16WHITE, u16BLACK);
            sprintf(buf, "  Ang:%.1f", PID.current_angle);
            TFTSPI_dir_P8X16Str(0, 4, buf, u16WHITE, u16BLACK);
            if(highlight_num == 2) TFTSPI_dir_P8X16Str(0, 5, "> BACK", u16YELLOW, u16BLACK);
            else                   TFTSPI_dir_P8X16Str(0, 5, "  BACK", u16WHITE, u16BLACK);
            break;

        case 2: // ================= PID 选择页 =================
            TFTSPI_dir_P8X16Str(0, 0, "=== PID Select ===", u16GREEN, u16BLACK);
            
            switch(highlight_num) {
                case 1: TFTSPI_dir_P8X16Str(0, 1, "> Outer Fuzzy", u16YELLOW, u16BLACK); break;
                case 2: TFTSPI_dir_P8X16Str(0, 2, "> Left  Inner", u16YELLOW, u16BLACK); break;
                case 3: TFTSPI_dir_P8X16Str(0, 3, "> Right Inner", u16YELLOW, u16BLACK); break;
                case 4: TFTSPI_dir_P8X16Str(0, 4, "> BACK",        u16YELLOW, u16BLACK); break;
            }
            
            if(highlight_num != 1) TFTSPI_dir_P8X16Str(0, 1, "  Outer Fuzzy", u16WHITE, u16BLACK);
            if(highlight_num != 2) TFTSPI_dir_P8X16Str(0, 2, "  Left  Inner", u16WHITE, u16BLACK);
            if(highlight_num != 3) TFTSPI_dir_P8X16Str(0, 3, "  Right Inner", u16WHITE, u16BLACK);
            if(highlight_num != 4) TFTSPI_dir_P8X16Str(0, 4, "  BACK",        u16WHITE, u16BLACK);
            break;

        case 3: // ================= Fuzzy 外环 =================
            TFTSPI_dir_P8X16Str(0, 0, "=== Fuzzy ===", u16GREEN, u16BLACK);
            if(write_ena) TFTSPI_dir_P8X16Str(14, 0, "[E]", u16RED, u16BLACK); 

            sprintf(buf, "%sBaseKp:%.2f", highlight_num == 1 ? "> ":"  ", KP_Base);
            TFTSPI_dir_P8X16Str(0, 1, buf, (highlight_num == 1) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "%sBaseKd:%.2f", highlight_num == 2 ? "> ":"  ", KD_Base);
            TFTSPI_dir_P8X16Str(0, 2, buf, (highlight_num == 2) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "%sFuzzKp:%.2f", highlight_num == 3 ? "> ":"  ", KP_Fuzzy);
            TFTSPI_dir_P8X16Str(0, 3, buf, (highlight_num == 3) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "%sFuzzKd:%.2f", highlight_num == 4 ? "> ":"  ", KD_Fuzzy);
            TFTSPI_dir_P8X16Str(0, 4, buf, (highlight_num == 4) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "%sErrMax:%.2f", highlight_num == 5 ? "> ":"  ", ERROR_MAX);
            TFTSPI_dir_P8X16Str(0, 5, buf, (highlight_num == 5) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            if(highlight_num == 6) TFTSPI_dir_P8X16Str(0, 6, "> BACK", u16YELLOW, u16BLACK);
            else                   TFTSPI_dir_P8X16Str(0, 6, "  BACK", u16WHITE, u16BLACK);
            break;

        case 4: // ================= 左轮速度环 =================
            TFTSPI_dir_P8X16Str(0, 0, "=== Left PID ===", u16GREEN, u16BLACK);
            if(write_ena) TFTSPI_dir_P8X16Str(14, 0, "[E]", u16RED, u16BLACK);

            sprintf(buf, "%sKp:%.2f", highlight_num == 1 ? "> ":"  ", PID_Speed_L.Kp);
            TFTSPI_dir_P8X16Str(0, 1, buf, (highlight_num == 1) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "%sKi:%.2f", highlight_num == 2 ? "> ":"  ", PID_Speed_L.Ki);
            TFTSPI_dir_P8X16Str(0, 2, buf, (highlight_num == 2) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "%sKd:%.2f", highlight_num == 3 ? "> ":"  ", PID_Speed_L.Kd);
            TFTSPI_dir_P8X16Str(0, 3, buf, (highlight_num == 3) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            if(highlight_num == 4) TFTSPI_dir_P8X16Str(0, 5, "> BACK", u16YELLOW, u16BLACK);
            else                   TFTSPI_dir_P8X16Str(0, 5, "  BACK", u16WHITE, u16BLACK);
            break;

        case 5: // ================= 右轮速度环 =================
            TFTSPI_dir_P8X16Str(0, 0, "=== Right PID ==", u16GREEN, u16BLACK);
            if(write_ena) TFTSPI_dir_P8X16Str(14, 0, "[E]", u16RED, u16BLACK);

            sprintf(buf, "%sKp:%.2f", highlight_num == 1 ? "> ":"  ", PID_Speed_R.Kp);
            TFTSPI_dir_P8X16Str(0, 1, buf, (highlight_num == 1) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "%sKi:%.2f", highlight_num == 2 ? "> ":"  ", PID_Speed_R.Ki);
            TFTSPI_dir_P8X16Str(0, 2, buf, (highlight_num == 2) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "%sKd:%.2f", highlight_num == 3 ? "> ":"  ", PID_Speed_R.Kd);
            TFTSPI_dir_P8X16Str(0, 3, buf, (highlight_num == 3) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            if(highlight_num == 4) TFTSPI_dir_P8X16Str(0, 5, "> BACK", u16YELLOW, u16BLACK);
            else                   TFTSPI_dir_P8X16Str(0, 5, "  BACK", u16WHITE, u16BLACK);
            break;
    }
}

void BayWatcher_Menu::refresh_menu(KeyOp Key) {
    switch(Key) {
        case KeyOp::OK: 
            if(write_ena == 1) {
                write_ena = 0; 
                write_pointer = NULL;
            } else {
                switch(menu_num) {
                    case 0: 
                        if(highlight_num == 1) { menu_num = 1; highlight_num = 1; }
                        else if(highlight_num == 2) { menu_num = 2; highlight_num = 1; }
                        else if(highlight_num == 3) { 
                            if(PID.is_running) BayWatcher_Stop_Car(); 
                            else BayWatcher_Start_Car(); 
                        }
                        break;
                    case 1: // 监控页跳转逻辑
                        if(highlight_num == 1) { 
                            write_ena = 1; 
                            write_pointer = &PID.base_target_speed; 
                            write_step = 1.0f; // 步进设置为 1
                        }
                        else if(highlight_num == 2) { 
                            menu_num = 0; 
                            highlight_num = 1; 
                        }
                        break;
                    case 2: 
                        if(highlight_num == 1) { menu_num = 3; highlight_num = 1; }
                        else if(highlight_num == 2) { menu_num = 4; highlight_num = 1; }
                        else if(highlight_num == 3) { menu_num = 5; highlight_num = 1; }
                        else if(highlight_num == 4) { menu_num = 0; highlight_num = 2; }
                        break;
                    case 3: 
                        if(highlight_num == 6) { menu_num = 2; highlight_num = 1; break; }
                        write_ena = 1;
                        if(highlight_num == 1) { write_pointer = &KP_Base;   write_step = 0.5f; }
                        else if(highlight_num == 2) { write_pointer = &KD_Base;   write_step = 0.5f; }
                        else if(highlight_num == 3) { write_pointer = &KP_Fuzzy;  write_step = 0.1f; }
                        else if(highlight_num == 4) { write_pointer = &KD_Fuzzy;  write_step = 1.0f; }
                        else if(highlight_num == 5) { write_pointer = &ERROR_MAX; write_step = 1.0f; }
                        break;
                    case 4: 
                        if(highlight_num == 4) { menu_num = 2; highlight_num = 2; break; }
                        write_ena = 1;
                        if(highlight_num == 1) { write_pointer = &PID_Speed_L.Kp; write_step = 0.1f; }
                        else if(highlight_num == 2) { write_pointer = &PID_Speed_L.Ki; write_step = 0.01f; }
                        else if(highlight_num == 3) { write_pointer = &PID_Speed_L.Kd; write_step = 0.1f; }
                        break;
                    case 5: 
                        if(highlight_num == 4) { menu_num = 2; highlight_num = 3; break; }
                        write_ena = 1;
                        if(highlight_num == 1) { write_pointer = &PID_Speed_R.Kp; write_step = 0.1f; }
                        else if(highlight_num == 2) { write_pointer = &PID_Speed_R.Ki; write_step = 0.01f; }
                        else if(highlight_num == 3) { write_pointer = &PID_Speed_R.Kd; write_step = 0.1f; }
                        break;
                }
            }
            break;

        case KeyOp::Cancel: 
        case KeyOp::Back:
            if(write_ena == 1) {
                write_ena = 0;
                write_pointer = NULL;
            } else {
                if(menu_num == 1 || menu_num == 2) { menu_num = 0; highlight_num = 1; }
                else if(menu_num >= 3) { menu_num = 2; highlight_num = 1; }
            }
            break;

        case KeyOp::Up: 
            if(write_ena == 1 && write_pointer != NULL) {
                *write_pointer += write_step; 
            } else {
                if(highlight_num > 1) highlight_num--; 
            }
            break;

        case KeyOp::Down: 
            if(write_ena == 1 && write_pointer != NULL) {
                *write_pointer -= write_step; 
            } else {
                uint8_t max_items = 1;
                if(menu_num == 0) max_items = 3;
                else if(menu_num == 1) max_items = 2; 
                else if(menu_num == 2) max_items = 4;
                else if(menu_num == 3) max_items = 6;
                else if(menu_num == 4 || menu_num == 5) max_items = 4;
                
                if(highlight_num < max_items) highlight_num++; 
            }
            break;
            
        default:
            break;
    }
    show(); 
}