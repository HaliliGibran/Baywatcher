#include "LQ_ATIM_PWM.hpp"
#include "LQ_GTIM_PWM.hpp"
#include "LQ_HW_GPIO.hpp"
#include "LQ_TFT18_dri.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cstring>
#include <memory>
#include <thread>

namespace {

// ===== 按键引脚：严格沿用 running 板 Key.cc =====
constexpr uint8_t KEY_UP_PIN = 44;
constexpr uint8_t KEY_DOWN_PIN = 45;
constexpr uint8_t KEY_OK_PIN = 80;
constexpr uint8_t KEY_CANCEL_PIN = 20;
constexpr uint8_t KEY_BACK_PIN = 17;

// ===== 电机参数：严格沿用 running 板 Motor.cc =====
constexpr uint8_t MOTOR_RIGHT_PWM_PIN = 81; // Motor1，右轮
constexpr uint8_t MOTOR_LEFT_PWM_PIN = 82;  // Motor2，左轮
constexpr uint8_t MOTOR_RIGHT_DIR_PIN = 21;
constexpr uint8_t MOTOR_LEFT_DIR_PIN = 22;
constexpr uint32_t MOTOR_PWM_PERIOD = 17000;
constexpr int16_t MOTOR_DUTY_HW_MAX = 10000;
constexpr int16_t MOTOR_DUTY_TEST_MAX = 5000; // 测试限幅 50%，需要满幅时再手动放开
constexpr int16_t MOTOR_DUTY_STEP = 500;

// ===== 电调参数：严格沿用 running 板 ESC.cc =====
constexpr uint8_t ESC_RIGHT_PWM_PIN = 89; // TIM2CH3，右负压
constexpr uint8_t ESC_LEFT_PWM_PIN = 77;  // TIM2CH4，左负压
constexpr uint32_t ESC_PWM_FREQ = 50;
constexpr uint16_t ESC_MIN_PWM = 500;
constexpr uint16_t ESC_MAX_PWM = 1000;
constexpr int16_t ESC_PERCENT_STEP = 5;
constexpr int16_t ESC_PERCENT_TEST_MAX = 50; // 测试限幅 50%，避免负压电调直接满输出

std::atomic_bool g_running{true};

enum class KeyEvent {
    None,
    Up,
    Down,
    OK,
    Cancel,
    Back,
};

enum class Page : uint8_t {
    Motor = 0,
    Esc,
};

enum class MotorItem : uint8_t {
    Enable = 1,
    LeftDuty,
    RightDuty,
    Stop,
};

enum class EscItem : uint8_t {
    Enable = 1,
    Percent,
    Stop,
};

struct DebouncedKey {
    std::unique_ptr<HWGpio> gpio;
    KeyEvent event = KeyEvent::None;
    bool last_raw = true;
    bool stable = true;
    std::chrono::steady_clock::time_point changed_at{};

    DebouncedKey(uint8_t pin, KeyEvent key_event)
        : gpio(new HWGpio(pin, GPIO_Mode_In)),
          event(key_event),
          changed_at(std::chrono::steady_clock::now())
    {
    }

    KeyEvent scan()
    {
        const bool now_raw = gpio->GetGpioValue();
        const auto now = std::chrono::steady_clock::now();

        if (now_raw != last_raw) {
            last_raw = now_raw;
            changed_at = now;
        }

        if ((now - changed_at) > std::chrono::milliseconds(20) && now_raw != stable) {
            stable = now_raw;
            if (!stable) {
                return event; // 低电平有效
            }
        }
        return KeyEvent::None;
    }
};

struct Hardware {
    std::unique_ptr<AtimPwm> motor_right_pwm;
    std::unique_ptr<AtimPwm> motor_left_pwm;
    std::unique_ptr<HWGpio> motor_right_dir;
    std::unique_ptr<HWGpio> motor_left_dir;

    std::unique_ptr<GtimPwm> esc_right_pwm;
    std::unique_ptr<GtimPwm> esc_left_pwm;

    std::unique_ptr<DebouncedKey> key_up;
    std::unique_ptr<DebouncedKey> key_down;
    std::unique_ptr<DebouncedKey> key_ok;
    std::unique_ptr<DebouncedKey> key_cancel;
    std::unique_ptr<DebouncedKey> key_back;
};

struct AppState {
    Page page = Page::Motor;
    uint8_t highlight = 1;
    bool editing = false;

    bool motor_enabled = false;
    int16_t motor_left_duty = 0;
    int16_t motor_right_duty = 0;

    bool esc_enabled = false;
    int16_t esc_percent = 0;
};

int16_t clamp_motor_duty(int16_t duty)
{
    const int16_t safe_max = std::min<int16_t>(MOTOR_DUTY_TEST_MAX, MOTOR_DUTY_HW_MAX);
    return std::max<int16_t>(-safe_max, std::min<int16_t>(safe_max, duty));
}

int16_t clamp_esc_percent(int16_t percent)
{
    return std::max<int16_t>(0, std::min<int16_t>(ESC_PERCENT_TEST_MAX, percent));
}

void reset_output_state(AppState& state)
{
    state.motor_enabled = false;
    state.motor_left_duty = 0;
    state.motor_right_duty = 0;
    state.esc_enabled = false;
    state.esc_percent = 0;
    state.editing = false;
}

void handle_signal(int)
{
    g_running = false;
}

void try_load_tft_modules()
{
    std::system("lsmod | grep -w TFT18_dev >/dev/null 2>&1 || insmod $(find /home -name 'TFT18_dev.ko' -print -quit) >/dev/null 2>&1 || true");
    std::system("lsmod | grep -w TFT18_dri >/dev/null 2>&1 || insmod $(find /home -name 'TFT18_dri.ko' -print -quit) >/dev/null 2>&1 || true");
}

void draw_line(uint8_t row, const char* text, uint16_t color = u16WHITE)
{
    char clipped[24];
    std::snprintf(clipped, sizeof(clipped), "%-20.20s", text);
    TFTSPI_dir_P8X16Str(0, row, clipped, color, u16BLACK);
}

void drawf(uint8_t row, uint16_t color, const char* fmt, ...)
{
    char buf[64];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    draw_line(row, buf, color);
}

uint16_t esc_percent_to_pwm(int16_t percent)
{
    percent = clamp_esc_percent(percent);
    uint16_t duty = static_cast<uint16_t>(ESC_MIN_PWM * (1.0f + percent * 0.01f));
    if (duty > ESC_MAX_PWM) duty = ESC_MAX_PWM;
    if (duty < ESC_MIN_PWM) duty = ESC_MIN_PWM;
    return duty;
}

void apply_motor(Hardware& hw, const AppState& state)
{
    if (!state.motor_enabled) {
        hw.motor_right_pwm->SetDutyCycle(0);
        hw.motor_left_pwm->SetDutyCycle(0);
        return;
    }

    const int16_t right = clamp_motor_duty(state.motor_right_duty);
    const int16_t left = clamp_motor_duty(state.motor_left_duty);

    // 方向逻辑沿用 running 的 Motor1_Set / Motor2_Set
    hw.motor_right_dir->SetGpioValue(right >= 0 ? 0 : 1);
    hw.motor_left_dir->SetGpioValue(left >= 0 ? 1 : 0);
    hw.motor_right_pwm->SetDutyCycle(static_cast<uint16_t>(std::abs(right)));
    hw.motor_left_pwm->SetDutyCycle(static_cast<uint16_t>(std::abs(left)));
}

void apply_esc(Hardware& hw, const AppState& state)
{
    const uint16_t duty = state.esc_enabled ? esc_percent_to_pwm(state.esc_percent) : ESC_MIN_PWM;
    hw.esc_left_pwm->SetDutyCycle(duty);
    hw.esc_right_pwm->SetDutyCycle(duty);
}

void stop_motor(Hardware& hw, AppState& state)
{
    state.motor_enabled = false;
    state.motor_left_duty = 0;
    state.motor_right_duty = 0;
    hw.motor_right_pwm->SetDutyCycle(0);
    hw.motor_left_pwm->SetDutyCycle(0);
}

void stop_esc(Hardware& hw, AppState& state)
{
    state.esc_enabled = false;
    state.esc_percent = 0;
    hw.esc_left_pwm->SetDutyCycle(ESC_MIN_PWM);
    hw.esc_right_pwm->SetDutyCycle(ESC_MIN_PWM);
}

void stop_all(Hardware& hw, AppState& state)
{
    stop_motor(hw, state);
    stop_esc(hw, state);
}

uint8_t max_highlight(Page page)
{
    return page == Page::Motor ? 4 : 3;
}

KeyEvent scan_keys(Hardware& hw)
{
    DebouncedKey* keys[] = {
        hw.key_up.get(),
        hw.key_down.get(),
        hw.key_ok.get(),
        hw.key_cancel.get(),
        hw.key_back.get(),
    };

    for (DebouncedKey* key : keys) {
        const KeyEvent event = key->scan();
        if (event != KeyEvent::None) return event;
    }
    return KeyEvent::None;
}

void switch_page(AppState& state)
{
    state.editing = false;
    state.highlight = 1;
    state.page = (state.page == Page::Motor) ? Page::Esc : Page::Motor;
}

void adjust_selected(AppState& state, int delta)
{
    if (state.page == Page::Motor) {
        if (state.highlight == static_cast<uint8_t>(MotorItem::LeftDuty)) {
            state.motor_left_duty = std::max<int16_t>(
                -MOTOR_DUTY_TEST_MAX,
                std::min<int16_t>(MOTOR_DUTY_TEST_MAX, state.motor_left_duty + delta * MOTOR_DUTY_STEP));
        } else if (state.highlight == static_cast<uint8_t>(MotorItem::RightDuty)) {
            state.motor_right_duty = std::max<int16_t>(
                -MOTOR_DUTY_TEST_MAX,
                std::min<int16_t>(MOTOR_DUTY_TEST_MAX, state.motor_right_duty + delta * MOTOR_DUTY_STEP));
        }
    } else {
        if (state.highlight == static_cast<uint8_t>(EscItem::Percent)) {
            state.esc_percent = std::max<int16_t>(
                0,
                std::min<int16_t>(ESC_PERCENT_TEST_MAX, state.esc_percent + delta * ESC_PERCENT_STEP));
        }
    }
}

void handle_event(KeyEvent event, Hardware& hw, AppState& state)
{
    if (event == KeyEvent::None) return;

    if (event == KeyEvent::Cancel) {
        stop_all(hw, state);
        state.editing = false;
        return;
    }

    if (event == KeyEvent::Back) {
        state.editing = false;
        switch_page(state);
        return;
    }

    if (state.editing) {
        if (event == KeyEvent::Up) adjust_selected(state, +1);
        if (event == KeyEvent::Down) adjust_selected(state, -1);
        if (event == KeyEvent::OK) state.editing = false;
        apply_motor(hw, state);
        apply_esc(hw, state);
        return;
    }

    if (event == KeyEvent::Up && state.highlight > 1) {
        state.highlight--;
        return;
    }
    if (event == KeyEvent::Down && state.highlight < max_highlight(state.page)) {
        state.highlight++;
        return;
    }

    if (event != KeyEvent::OK) return;

    if (state.page == Page::Motor) {
        switch (static_cast<MotorItem>(state.highlight)) {
        case MotorItem::Enable:
            state.motor_enabled = !state.motor_enabled;
            apply_motor(hw, state);
            break;
        case MotorItem::LeftDuty:
        case MotorItem::RightDuty:
            state.editing = true;
            break;
        case MotorItem::Stop:
            stop_motor(hw, state);
            break;
        }
    } else {
        switch (static_cast<EscItem>(state.highlight)) {
        case EscItem::Enable:
            state.esc_enabled = !state.esc_enabled;
            apply_esc(hw, state);
            break;
        case EscItem::Percent:
            state.editing = true;
            break;
        case EscItem::Stop:
            stop_esc(hw, state);
            break;
        }
    }
}

uint16_t item_color(const AppState& state, uint8_t item)
{
    if (state.highlight != item) return u16WHITE;
    return state.editing ? u16RED : u16YELLOW;
}

const char* cursor(const AppState& state, uint8_t item)
{
    return state.highlight == item ? "> " : "  ";
}

void draw_motor_page(const AppState& state)
{
    TFTSPI_dir_P8X16Str(0, 0, "=== Motor PWM ===", u16GREEN, u16BLACK);
    if (state.editing) TFTSPI_dir_P8X16Str(14, 0, "[E]", u16RED, u16BLACK);

    drawf(1, item_color(state, 1), "%sOutput:%s",
          cursor(state, 1), state.motor_enabled ? "ON" : "OFF");
    drawf(2, item_color(state, 2), "%sLeft :%d",
          cursor(state, 2), state.motor_left_duty);
    drawf(3, item_color(state, 3), "%sRight:%d",
          cursor(state, 3), state.motor_right_duty);
    drawf(4, item_color(state, 4), "%sSTOP Motor", cursor(state, 4));
    drawf(5, u16WHITE, "  Limit:+/-%d", MOTOR_DUTY_TEST_MAX);
    draw_line(6, "UP/DN move/change");
    draw_line(7, "BACK page CAN stop", u16CYAN);
}

void draw_esc_page(const AppState& state)
{
    TFTSPI_dir_P8X16Str(0, 0, "=== ESC PWM ===", u16GREEN, u16BLACK);
    if (state.editing) TFTSPI_dir_P8X16Str(14, 0, "[E]", u16RED, u16BLACK);

    drawf(1, item_color(state, 1), "%sOutput:%s",
          cursor(state, 1), state.esc_enabled ? "ON" : "OFF");
    drawf(2, item_color(state, 2), "%sPercent:%d%%",
          cursor(state, 2), state.esc_percent);
    drawf(3, u16WHITE, "  PWM:%u", esc_percent_to_pwm(state.esc_percent));
    drawf(4, item_color(state, 3), "%sSTOP ESC", cursor(state, 3));
    drawf(5, u16WHITE, "  Limit:%d%%", ESC_PERCENT_TEST_MAX);
    draw_line(6, "UP/DN move/change");
    draw_line(7, "BACK page CAN stop", u16CYAN);
}

void draw_screen(const AppState& state)
{
    TFTSPI_dir_cls(u16BLACK);
    if (state.page == Page::Motor) {
        draw_motor_page(state);
    } else {
        draw_esc_page(state);
    }
    TFTSPI_dir_flush();
}

void init_hardware(Hardware& hw)
{
    try_load_tft_modules();

    TFTSPI_dri_init(1);
    TFTSPI_dir_cls(u16BLACK);
    draw_line(0, "Blindbox PWM Test", u16GREEN);
    draw_line(1, "initializing...");
    TFTSPI_dir_flush();

    hw.motor_right_pwm.reset(new AtimPwm(MOTOR_RIGHT_PWM_PIN, 1, LS_ATIM_INVERSED, MOTOR_PWM_PERIOD, 0));
    hw.motor_left_pwm.reset(new AtimPwm(MOTOR_LEFT_PWM_PIN, 2, LS_ATIM_INVERSED, MOTOR_PWM_PERIOD, 0));
    hw.motor_right_dir.reset(new HWGpio(MOTOR_RIGHT_DIR_PIN, GPIO_Mode_Out));
    hw.motor_left_dir.reset(new HWGpio(MOTOR_LEFT_DIR_PIN, GPIO_Mode_Out));
    hw.motor_right_pwm->Enable();
    hw.motor_left_pwm->Enable();
    hw.motor_right_dir->SetGpioValue(0);
    hw.motor_left_dir->SetGpioValue(1);
    hw.motor_right_pwm->SetDutyCycle(0);
    hw.motor_left_pwm->SetDutyCycle(0);

    hw.esc_right_pwm.reset(new GtimPwm(ESC_RIGHT_PWM_PIN, 3, LS_GTIM_INVERSED, ESC_PWM_FREQ, ESC_MIN_PWM));
    hw.esc_left_pwm.reset(new GtimPwm(ESC_LEFT_PWM_PIN, 4, LS_GTIM_INVERSED, ESC_PWM_FREQ, ESC_MIN_PWM, 0b01));
    hw.esc_right_pwm->Enable();
    hw.esc_left_pwm->Enable();
    hw.esc_right_pwm->SetDutyCycle(ESC_MIN_PWM);
    hw.esc_left_pwm->SetDutyCycle(ESC_MIN_PWM);

    hw.key_up.reset(new DebouncedKey(KEY_UP_PIN, KeyEvent::Up));
    hw.key_down.reset(new DebouncedKey(KEY_DOWN_PIN, KeyEvent::Down));
    hw.key_ok.reset(new DebouncedKey(KEY_OK_PIN, KeyEvent::OK));
    hw.key_cancel.reset(new DebouncedKey(KEY_CANCEL_PIN, KeyEvent::Cancel));
    hw.key_back.reset(new DebouncedKey(KEY_BACK_PIN, KeyEvent::Back));
}

} // namespace

int main()
{
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    Hardware hw;
    AppState state;

    reset_output_state(state);
    init_hardware(hw);
    stop_all(hw, state);
    draw_screen(state);

    auto last_draw = std::chrono::steady_clock::now();
    while (g_running) {
        const KeyEvent event = scan_keys(hw);
        if (event != KeyEvent::None) {
            handle_event(event, hw, state);
            draw_screen(state);
            last_draw = std::chrono::steady_clock::now();
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_draw > std::chrono::milliseconds(500)) {
            draw_screen(state);
            last_draw = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    stop_all(hw, state);
    TFTSPI_dir_cls(u16BLACK);
    draw_line(0, "PWM test stopped", u16YELLOW);
    TFTSPI_dir_flush();
    return 0;
}
