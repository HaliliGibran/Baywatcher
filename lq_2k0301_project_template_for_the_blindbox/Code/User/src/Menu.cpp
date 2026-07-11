#include "main.hpp"

#include <stdio.h>

namespace {

constexpr int16_t MOTOR_DUTY_LIMIT = 7000;
constexpr int16_t MOTOR_DUTY_STEP = 500;
constexpr int16_t ESC_PERCENT_LIMIT = 80;
constexpr int16_t ESC_PERCENT_STEP = 5;

int16_t motor_left_duty = 0;
int16_t motor_right_duty = 0;
int16_t esc_percent = 0;
bool motor_output_enable = false;
bool esc_output_enable = false;

int16_t limit_i16(int16_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

uint16_t esc_percent_to_pwm(int16_t percent)
{
    percent = limit_i16(percent, 0, ESC_PERCENT_LIMIT);
    return static_cast<uint16_t>(500 * (1.0f + percent * 0.01f));
}

void apply_motor_output()
{
    if (!motor_output_enable) {
        motor_sys.Stop();
        return;
    }

    motor_left_duty = limit_i16(motor_left_duty, -MOTOR_DUTY_LIMIT, MOTOR_DUTY_LIMIT);
    motor_right_duty = limit_i16(motor_right_duty, -MOTOR_DUTY_LIMIT, MOTOR_DUTY_LIMIT);
    motor_sys.Motor2_Set(motor_left_duty);   // 左轮：running 中 Motor2
    motor_sys.Motor1_Set(motor_right_duty);  // 右轮：running 中 Motor1
}

void apply_esc_output()
{
    if (!esc_output_enable) {
        esc_sys.stop();
        return;
    }

    esc_percent = limit_i16(esc_percent, 0, ESC_PERCENT_LIMIT);
    esc_sys.Esc1_Set(esc_percent); // 左负压 IO77
    esc_sys.Esc2_Set(esc_percent); // 右负压 IO89
}

void stop_all_output()
{
    motor_output_enable = false;
    esc_output_enable = false;
    motor_left_duty = 0;
    motor_right_duty = 0;
    esc_percent = 0;
    motor_sys.Stop();
    esc_sys.stop();
}

} // namespace

void BayWatcher_Menu::init()
{
    menu_num = 0;
    highlight_num = 1;
    write_ena = 0;
    write_pointer = NULL;
    write_pointer_b = NULL;
    write_step = 1.0f;
    data_max = 0.0f;
    data_min = 0.0f;

    TFTSPI_dri_init(1);
    stop_all_output();
    show();
}

void BayWatcher_Menu::show()
{
    TFTSPI_dir_cls(u16BLACK);
    char buf[32];

    switch (menu_num) {
        case 0: // ================= 电机 PWM =================
            TFTSPI_dir_P8X16Str(0, 0, "=== Motor PWM ===", u16GREEN, u16BLACK);
            if (write_ena) TFTSPI_dir_P8X16Str(14, 0, "[E]", u16RED, u16BLACK);

            sprintf(buf, "%sOutput:%s", highlight_num == 1 ? "> " : "  ",
                    motor_output_enable ? "ON" : "OFF");
            TFTSPI_dir_P8X16Str(0, 1, buf,
                (highlight_num == 1) ? u16YELLOW : u16WHITE, u16BLACK);

            sprintf(buf, "%sLeft :%d", highlight_num == 2 ? "> " : "  ", motor_left_duty);
            TFTSPI_dir_P8X16Str(0, 2, buf,
                (highlight_num == 2) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "%sRight:%d", highlight_num == 3 ? "> " : "  ", motor_right_duty);
            TFTSPI_dir_P8X16Str(0, 3, buf,
                (highlight_num == 3) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "%sSTOP Motor", highlight_num == 4 ? "> " : "  ");
            TFTSPI_dir_P8X16Str(0, 4, buf,
                (highlight_num == 4) ? u16YELLOW : u16WHITE, u16BLACK);

            sprintf(buf, "  Limit:+/-%d", MOTOR_DUTY_LIMIT);
            TFTSPI_dir_P8X16Str(0, 5, buf, u16WHITE, u16BLACK);
            TFTSPI_dir_P8X16Str(0, 6, "  BACK:ESC page", u16CYAN, u16BLACK);
            TFTSPI_dir_P8X16Str(0, 7, "  CAN:STOP ALL", u16CYAN, u16BLACK);
            break;

        case 1: // ================= 电调 PWM =================
            TFTSPI_dir_P8X16Str(0, 0, "=== ESC PWM ===", u16GREEN, u16BLACK);
            if (write_ena) TFTSPI_dir_P8X16Str(14, 0, "[E]", u16RED, u16BLACK);

            sprintf(buf, "%sOutput:%s", highlight_num == 1 ? "> " : "  ",
                    esc_output_enable ? "ON" : "OFF");
            TFTSPI_dir_P8X16Str(0, 1, buf,
                (highlight_num == 1) ? u16YELLOW : u16WHITE, u16BLACK);

            sprintf(buf, "%sPercent:%d%%", highlight_num == 2 ? "> " : "  ", esc_percent);
            TFTSPI_dir_P8X16Str(0, 2, buf,
                (highlight_num == 2) ? (write_ena ? u16RED : u16YELLOW) : u16WHITE, u16BLACK);

            sprintf(buf, "  PWM:%u", esc_percent_to_pwm(esc_percent));
            TFTSPI_dir_P8X16Str(0, 3, buf, u16WHITE, u16BLACK);

            sprintf(buf, "%sSTOP ESC", highlight_num == 3 ? "> " : "  ");
            TFTSPI_dir_P8X16Str(0, 4, buf,
                (highlight_num == 3) ? u16YELLOW : u16WHITE, u16BLACK);

            sprintf(buf, "  Limit:%d%%", ESC_PERCENT_LIMIT);
            TFTSPI_dir_P8X16Str(0, 5, buf, u16WHITE, u16BLACK);
            TFTSPI_dir_P8X16Str(0, 6, "  BACK:Motor page", u16CYAN, u16BLACK);
            TFTSPI_dir_P8X16Str(0, 7, "  CAN:STOP ALL", u16CYAN, u16BLACK);
            break;
    }
}

void BayWatcher_Menu::refresh_menu(KeyOp key)
{
    switch (key) {
        case KeyOp::OK:
            if (write_ena) {
                write_ena = 0;
            } else if (menu_num == 0) {
                if (highlight_num == 1) {
                    motor_output_enable = !motor_output_enable;
                    apply_motor_output();
                } else if (highlight_num == 2 || highlight_num == 3) {
                    write_ena = 1;
                } else if (highlight_num == 4) {
                    motor_output_enable = false;
                    motor_left_duty = 0;
                    motor_right_duty = 0;
                    motor_sys.Stop();
                }
            } else if (menu_num == 1) {
                if (highlight_num == 1) {
                    esc_output_enable = !esc_output_enable;
                    apply_esc_output();
                } else if (highlight_num == 2) {
                    write_ena = 1;
                } else if (highlight_num == 3) {
                    esc_output_enable = false;
                    esc_percent = 0;
                    esc_sys.stop();
                }
            }
            break;

        case KeyOp::Cancel:
            if (menu_num == 0) {
                motor_output_enable = false;
                motor_left_duty = 0;
                motor_right_duty = 0;
                motor_sys.Stop();
            } else {
                esc_output_enable = false;
                esc_percent = 0;
                esc_sys.stop();
            }
            write_ena = 0;
            break;

        case KeyOp::Back:
            write_ena = 0;
            menu_num = (menu_num == 0) ? 1 : 0;
            highlight_num = 1;
            break;

        case KeyOp::Up:
            if (write_ena) {
                if (menu_num == 0 && highlight_num == 2) {
                    motor_left_duty = limit_i16(motor_left_duty + MOTOR_DUTY_STEP,
                                                -MOTOR_DUTY_LIMIT, MOTOR_DUTY_LIMIT);
                    apply_motor_output();
                } else if (menu_num == 0 && highlight_num == 3) {
                    motor_right_duty = limit_i16(motor_right_duty + MOTOR_DUTY_STEP,
                                                 -MOTOR_DUTY_LIMIT, MOTOR_DUTY_LIMIT);
                    apply_motor_output();
                } else if (menu_num == 1 && highlight_num == 2) {
                    esc_percent = limit_i16(esc_percent + ESC_PERCENT_STEP, 0, ESC_PERCENT_LIMIT);
                    apply_esc_output();
                }
            } else {
                if (highlight_num > 1) highlight_num--;
            }
            break;

        case KeyOp::Down:
            if (write_ena) {
                if (menu_num == 0 && highlight_num == 2) {
                    motor_left_duty = limit_i16(motor_left_duty - MOTOR_DUTY_STEP,
                                                -MOTOR_DUTY_LIMIT, MOTOR_DUTY_LIMIT);
                    apply_motor_output();
                } else if (menu_num == 0 && highlight_num == 3) {
                    motor_right_duty = limit_i16(motor_right_duty - MOTOR_DUTY_STEP,
                                                 -MOTOR_DUTY_LIMIT, MOTOR_DUTY_LIMIT);
                    apply_motor_output();
                } else if (menu_num == 1 && highlight_num == 2) {
                    esc_percent = limit_i16(esc_percent - ESC_PERCENT_STEP, 0, ESC_PERCENT_LIMIT);
                    apply_esc_output();
                }
            } else {
                uint8_t max_items = (menu_num == 0) ? 4 : 3;
                if (highlight_num < max_items) highlight_num++;
            }
            break;

        default:
            break;
    }

    show();
}
