#include "main.hpp"
#include <stdio.h> 

void BayWatcher_Menu::init() {
    menu_num = 0;
    highlight_num = 1;
    write_ena = 0;
    write_pointer = NULL;
    write_pointer_b = NULL;
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
                case 4: TFTSPI_dir_P8X16Str(0, 4, "> 4.ESC Config", u16YELLOW, u16BLACK); break;
            }
            
            if(highlight_num != 1) TFTSPI_dir_P8X16Str(0, 1, "  1.Monitor", u16WHITE, u16BLACK);
            if(highlight_num != 2) TFTSPI_dir_P8X16Str(0, 2, "  2.PID Select", u16WHITE, u16BLACK);
            if(highlight_num != 3) {
                if(PID.is_running) TFTSPI_dir_P8X16Str(0, 3, "  3.Stop Car", u16WHITE, u16BLACK);
                else               TFTSPI_dir_P8X16Str(0, 3, "  3.Start Car", u16WHITE, u16BLACK);
            }
            if(highlight_num != 4) TFTSPI_dir_P8X16Str(0, 4, "  4.ESC Config", u16WHITE, u16BLACK);
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
                case 1: TFTSPI_dir_P8X16Str(0, 1, "> Outer Cube ", u16YELLOW, u16BLACK); break;
                case 2: TFTSPI_dir_P8X16Str(0, 2, "> Left  Inner", u16YELLOW, u16BLACK); break;
                case 3: TFTSPI_dir_P8X16Str(0, 3, "> Right Inner", u16YELLOW, u16BLACK); break;
                case 4: TFTSPI_dir_P8X16Str(0, 4, "> BACK",        u16YELLOW, u16BLACK); break;
            }
            
            if(highlight_num != 1) TFTSPI_dir_P8X16Str(0, 1, "  Outer Cube ", u16WHITE, u16BLACK);
            if(highlight_num != 2) TFTSPI_dir_P8X16Str(0, 2, "  Left  Inner", u16WHITE, u16BLACK);
            if(highlight_num != 3) TFTSPI_dir_P8X16Str(0, 3, "  Right Inner", u16WHITE, u16BLACK);
            if(highlight_num != 4) TFTSPI_dir_P8X16Str(0, 4, "  BACK",        u16WHITE, u16BLACK);
            break;

        case 3: // ================= Cube 外环 =================
            TFTSPI_dir_P8X16Str(0, 0, "=== Cube Outer ===", u16GREEN, u16BLACK);
            if(write_ena) TFTSPI_dir_P8X16Str(14, 0, "[E]", u16RED, u16BLACK); 

            sprintf(buf, "%sKp_a:%.3f", highlight_num == 1 ? "> ":"  ", PID_Cube.Kp_a);
            TFTSPI_dir_P8X16Str(0, 1, buf, (highlight_num == 1) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "%sKp_b:%.5f", highlight_num == 2 ? "> ":"  ", PID_Cube.Kp_b);
            TFTSPI_dir_P8X16Str(0, 2, buf, (highlight_num == 2) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "%sKi  :%.3f", highlight_num == 3 ? "> ":"  ", PID_Cube.Ki);
            TFTSPI_dir_P8X16Str(0, 3, buf, (highlight_num == 3) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "%sKd_a:%.3f", highlight_num == 4 ? "> ":"  ", PID_Cube.Kd_a);
            TFTSPI_dir_P8X16Str(0, 4, buf, (highlight_num == 4) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "%sKd_b:%.3f", highlight_num == 5 ? "> ":"  ", PID_Cube.Kd_b);
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
        case 6: // ================= ESC/Startup Config =================
            TFTSPI_dir_P8X16Str(0, 0, "=== ESC Config ===", u16GREEN, u16BLACK);
            if(write_ena) TFTSPI_dir_P8X16Str(14, 0, "[E]", u16RED, u16BLACK);

            sprintf(buf, "%sESCDiff:%d", highlight_num == 1 ? "> ":"  ", esc_sys.enable_esc_diff ? 1 : 0);
            TFTSPI_dir_P8X16Str(0, 1, buf, (highlight_num == 1) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "%sDiffRt:%.2f", highlight_num == 2 ? "> ":"  ", esc_sys.esc_diff_ratio);
            TFTSPI_dir_P8X16Str(0, 2, buf, (highlight_num == 2) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            if(highlight_num == 3) TFTSPI_dir_P8X16Str(0, 3, "> BACK", u16YELLOW, u16BLACK);
            else                   TFTSPI_dir_P8X16Str(0, 3, "  BACK", u16WHITE, u16BLACK);
            break;
    }
}

void BayWatcher_Menu::refresh_menu(KeyOp Key) {
    switch(Key) {
        case KeyOp::OK: 
            if(write_ena == 1) {
                write_ena = 0; 
                write_pointer = NULL;
                write_pointer_b = NULL;
            } else {
                switch(menu_num) {
                    case 0: 
                        if(highlight_num == 1) { menu_num = 1; highlight_num = 1; }
                        else if(highlight_num == 2) { menu_num = 2; highlight_num = 1; }
                        else if(highlight_num == 3) { 
                            if(PID.is_running) BayWatcher_Stop_Car(); 
                            else BayWatcher_Start_Car(); 
                        }
                        else if(highlight_num == 4) { menu_num = 6; highlight_num = 1; }
                        break;
                    case 1: // 监控页跳转逻辑
                        if(highlight_num == 1) { 
                            write_ena = 1; 
                            write_pointer = &PID.base_target_speed; 
                            write_step = 1.0f; // 步进设置为 1
                            data_max = 500.0f;
                            data_min = -100.0f;
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
                        data_min = 0.0f;
                        if(highlight_num == 1) { write_pointer = &PID_Cube.Kp_a;  write_step = 0.005f; data_max = 50.0f; }
                        else if(highlight_num == 2) { write_pointer = &PID_Cube.Kp_b;  write_step = 0.0001f; data_max = 10.0f; }
                        else if(highlight_num == 3) { write_pointer = &PID_Cube.Ki;    write_step = 0.005f; data_max = 50.0f; }
                        else if(highlight_num == 4) { write_pointer = &PID_Cube.Kd_a;  write_step = 0.05f; data_max = 50.0f; }
                        else if(highlight_num == 5) { write_pointer = &PID_Cube.Kd_b;  write_step = 0.05f; data_max = 50.0f; }
                        break;
                    case 4: 
                        if(highlight_num == 4) { menu_num = 2; highlight_num = 2; break; }
                        write_ena = 1;
                        data_min = 0.0f;
                        if(highlight_num == 1) { write_pointer = &PID_Speed_L.Kp; write_step = 0.1f; data_max = 500.0f; }
                        else if(highlight_num == 2) { write_pointer = &PID_Speed_L.Ki; write_step = 0.01f; data_max = 50.0f; }
                        else if(highlight_num == 3) { write_pointer = &PID_Speed_L.Kd; write_step = 0.1f; data_max = 50.0f; }
                        break;
                    case 5: 
                        if(highlight_num == 4) { menu_num = 2; highlight_num = 3; break; }
                        write_ena = 1;
                        data_min = 0.0f;
                        if(highlight_num == 1) { write_pointer = &PID_Speed_R.Kp; write_step = 0.1f; data_max = 500.0f; }
                        else if(highlight_num == 2) { write_pointer = &PID_Speed_R.Ki; write_step = 0.01f; data_max = 50.0f; }
                        else if(highlight_num == 3) { write_pointer = &PID_Speed_R.Kd; write_step = 0.1f; data_max = 50.0f; }
                        break;
                    case 6: 
                        if(highlight_num == 3) { menu_num = 0; highlight_num = 4; break; }
                        write_ena = 1;
                        data_min = 0.0f;
                        if(highlight_num == 1) { write_pointer_b = &esc_sys.enable_esc_diff; write_pointer = NULL; } 
                        else if(highlight_num == 2) { write_pointer = &esc_sys.esc_diff_ratio; write_pointer_b = NULL; write_step = 0.1f; data_max = 10.0f; }
                        break;
                }
            }
            break;

        case KeyOp::Cancel: 
        case KeyOp::Back:
            if(write_ena == 1) {
                write_ena = 0;
                write_pointer = NULL;
                write_pointer_b = NULL;
            } else {
                if(menu_num == 1 || menu_num == 2 || menu_num == 6) { menu_num = 0; highlight_num = 1; }
                else if(menu_num >= 3 && menu_num <= 5) { menu_num = 2; highlight_num = 1; }
            }
            break;

        case KeyOp::Up: 
            if(write_ena == 1) {
                if (write_pointer != NULL) {
                    *write_pointer += write_step; 
                    if (*write_pointer > data_max) {
                        *write_pointer = data_max;
                    }
                } else if (write_pointer_b != NULL) {
                    *write_pointer_b = !(*write_pointer_b);
                }
            } else {
                if(highlight_num > 1) highlight_num--; 
            }
            break;

        case KeyOp::Down: 
            if(write_ena == 1) {
                if (write_pointer != NULL) {
                    *write_pointer -= write_step; 
                    if (*write_pointer < data_min) {
                        *write_pointer = data_min;
                    }
                } else if (write_pointer_b != NULL) {
                    *write_pointer_b = !(*write_pointer_b);
                }
            } else {
                uint8_t max_items = 1;
                if(menu_num == 0) max_items = 4;
                else if(menu_num == 1) max_items = 2; 
                else if(menu_num == 2) max_items = 4;
                else if(menu_num == 3) max_items = 6;
                else if(menu_num == 4 || menu_num == 5) max_items = 4;
                else if(menu_num == 6) max_items = 3;
                
                if(highlight_num < max_items) highlight_num++; 
            }
            break;
            
        default:
            break;
    }
    show(); 
}