#include "Buzzer.h"
#include "IMU.h"
#include "LQ_ATIM_PWM.hpp"
#include "LQ_GTIM_PWM.hpp"
#include "LQ_HW_GPIO.hpp"
#include "LQ_PWM_ENCODER.hpp"
#include "LQ_TFT18_dri.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cstring>
#include <cstdarg>
#include <memory>
#include <thread>

namespace {

constexpr uint8_t KEY_UP_PIN = 44;
constexpr uint8_t KEY_DOWN_PIN = 45;
constexpr uint8_t KEY_OK_PIN = 80;
constexpr uint8_t KEY_CANCEL_PIN = 20;
constexpr uint8_t KEY_BACK_PIN = 17;

constexpr int16_t MOTOR_DUTY_STEP = 500;
constexpr int16_t MOTOR_DUTY_MAX = 5000;   // 测试默认限幅 50%，避免误伤
constexpr int16_t ESC_PERCENT_STEP = 5;
constexpr int16_t ESC_PERCENT_MAX = 50;    // 测试默认限幅 50%
constexpr uint16_t ESC_MIN_PWM = 500;

std::atomic_bool g_running{true};

enum class KeyEvent {
    None,
    Up,
    Down,
    OK,
    Cancel,
    Back,
};

enum class TestPage : uint8_t {
    Buzzer = 0,
    Encoder,
    MotorPwm,
    EscPwm,
    Keys,
    Imu,
    Count,
};

struct DebouncedKey {
    const char* name;
    uint8_t pin;
    KeyEvent event;
    std::unique_ptr<HWGpio> gpio;
    bool last_raw = true;
    bool stable = true;
    std::chrono::steady_clock::time_point changed_at{};

    DebouncedKey(const char* key_name, uint8_t key_pin, KeyEvent key_event)
        : name(key_name), pin(key_pin), event(key_event),
          gpio(new HWGpio(key_pin, GPIO_Mode_In)),
          changed_at(std::chrono::steady_clock::now())
    {
    }

    bool raw_level() const
    {
        return gpio->GetGpioValue();
    }

    KeyEvent scan()
    {
        const bool now_raw = raw_level();
        const auto now = std::chrono::steady_clock::now();

        if (now_raw != last_raw) {
            changed_at = now;
            last_raw = now_raw;
        }

        if (now - changed_at > std::chrono::milliseconds(20) && now_raw != stable) {
            stable = now_raw;
            if (!stable) {
                return event; // 按键低电平有效
            }
        }
        return KeyEvent::None;
    }
};

struct Hardware {
    std::unique_ptr<LS_PwmEncoder> enc_l;
    std::unique_ptr<LS_PwmEncoder> enc_r;

    std::unique_ptr<AtimPwm> motor_l_pwm;
    std::unique_ptr<AtimPwm> motor_r_pwm;
    std::unique_ptr<HWGpio> motor_l_dir;
    std::unique_ptr<HWGpio> motor_r_dir;

    std::unique_ptr<GtimPwm> esc_l_pwm;
    std::unique_ptr<GtimPwm> esc_r_pwm;

    std::unique_ptr<DebouncedKey> key_up;
    std::unique_ptr<DebouncedKey> key_down;
    std::unique_ptr<DebouncedKey> key_ok;
    std::unique_ptr<DebouncedKey> key_cancel;
    std::unique_ptr<DebouncedKey> key_back;
};

struct AppState {
    TestPage page = TestPage::Buzzer;
    bool motor_enabled = false;
    bool esc_enabled = false;
    int16_t motor_left_duty = 0;
    int16_t motor_right_duty = 0;
    int16_t esc_percent = 0;
    uint32_t buzzer_ms = 300;
    float enc_l = 0.0f;
    float enc_r = 0.0f;
};

void handle_signal(int)
{
    g_running = false;
}

void try_load_module(const char* module_name)
{
    char cmd[256];
    std::snprintf(cmd, sizeof(cmd),
                  "lsmod | grep -w '%s' >/dev/null 2>&1 || "
                  "insmod $(find /home -name '%s.ko' -print -quit) >/dev/null 2>&1 || true",
                  module_name, module_name);
    std::system(cmd);
}

void try_load_required_modules()
{
    try_load_module("lq_i2c_all_dev");
    try_load_module("lq_i2c_mpu6050_drv");
    try_load_module("TFT18_dev");
    try_load_module("TFT18_dri");
}

const char* page_title(TestPage page)
{
    switch (page) {
    case TestPage::Buzzer: return "Buzzer";
    case TestPage::Encoder: return "Encoder";
    case TestPage::MotorPwm: return "Motor PWM";
    case TestPage::EscPwm: return "ESC PWM";
    case TestPage::Keys: return "Keys";
    case TestPage::Imu: return "IMU";
    default: return "-";
    }
}

uint8_t page_index(TestPage page)
{
    return static_cast<uint8_t>(page);
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

void motor_stop(Hardware& hw, AppState& state)
{
    state.motor_enabled = false;
    if (hw.motor_l_pwm) hw.motor_l_pwm->SetDutyCycle(0);
    if (hw.motor_r_pwm) hw.motor_r_pwm->SetDutyCycle(0);
}

void esc_stop(Hardware& hw, AppState& state)
{
    state.esc_enabled = false;
    if (hw.esc_l_pwm) hw.esc_l_pwm->SetDutyCycle(ESC_MIN_PWM);
    if (hw.esc_r_pwm) hw.esc_r_pwm->SetDutyCycle(ESC_MIN_PWM);
}

void apply_motor_pwm(Hardware& hw, const AppState& state)
{
    const int16_t left = std::max<int16_t>(-MOTOR_DUTY_MAX, std::min<int16_t>(MOTOR_DUTY_MAX, state.motor_left_duty));
    const int16_t right = std::max<int16_t>(-MOTOR_DUTY_MAX, std::min<int16_t>(MOTOR_DUTY_MAX, state.motor_right_duty));

    hw.motor_l_dir->SetGpioValue(left >= 0 ? 1 : 0);  // 与 running 的 Motor2 左轮方向一致
    hw.motor_r_dir->SetGpioValue(right >= 0 ? 0 : 1); // 与 running 的 Motor1 右轮方向一致
    hw.motor_l_pwm->SetDutyCycle(static_cast<uint16_t>(std::abs(left)));
    hw.motor_r_pwm->SetDutyCycle(static_cast<uint16_t>(std::abs(right)));
}

void apply_esc_pwm(Hardware& hw, const AppState& state)
{
    const int16_t pct = std::max<int16_t>(0, std::min<int16_t>(ESC_PERCENT_MAX, state.esc_percent));
    const uint16_t duty = static_cast<uint16_t>(ESC_MIN_PWM * (1.0f + pct * 0.01f));
    hw.esc_l_pwm->SetDutyCycle(duty);  // 左负压 IO77
    hw.esc_r_pwm->SetDutyCycle(duty);  // 右负压 IO89
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
        KeyEvent event = key->scan();
        if (event != KeyEvent::None) {
            return event;
        }
    }
    return KeyEvent::None;
}

void handle_event(KeyEvent event, Hardware& hw, AppState& state)
{
    if (event == KeyEvent::None) return;

    if (event == KeyEvent::Cancel || event == KeyEvent::Back) {
        motor_stop(hw, state);
        esc_stop(hw, state);
        buzzer_sys.off();
        return;
    }

    switch (state.page) {
    case TestPage::Buzzer:
        if (event == KeyEvent::OK) buzzer_sys.beep(state.buzzer_ms);
        break;

    case TestPage::Encoder:
        if (event == KeyEvent::OK && hw.enc_l && hw.enc_r) {
            hw.enc_l->ResetCounter();
            hw.enc_r->ResetCounter();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            hw.enc_l->CloseResCounter();
            hw.enc_r->CloseResCounter();
        }
        break;

    case TestPage::MotorPwm:
        if (event == KeyEvent::Up) {
            state.motor_left_duty = std::min<int16_t>(MOTOR_DUTY_MAX, state.motor_left_duty + MOTOR_DUTY_STEP);
            state.motor_right_duty = std::min<int16_t>(MOTOR_DUTY_MAX, state.motor_right_duty + MOTOR_DUTY_STEP);
        }
        if (event == KeyEvent::Down) {
            state.motor_left_duty = std::max<int16_t>(-MOTOR_DUTY_MAX, state.motor_left_duty - MOTOR_DUTY_STEP);
            state.motor_right_duty = std::max<int16_t>(-MOTOR_DUTY_MAX, state.motor_right_duty - MOTOR_DUTY_STEP);
        }
        if (event == KeyEvent::OK) state.motor_enabled = !state.motor_enabled;
        if (state.motor_enabled) apply_motor_pwm(hw, state);
        else motor_stop(hw, state);
        break;

    case TestPage::EscPwm:
        if (event == KeyEvent::Up) state.esc_percent = std::min<int16_t>(ESC_PERCENT_MAX, state.esc_percent + ESC_PERCENT_STEP);
        if (event == KeyEvent::Down) state.esc_percent = std::max<int16_t>(0, state.esc_percent - ESC_PERCENT_STEP);
        if (event == KeyEvent::OK) state.esc_enabled = !state.esc_enabled;
        if (state.esc_enabled) apply_esc_pwm(hw, state);
        else esc_stop(hw, state);
        break;

    case TestPage::Keys:
    case TestPage::Imu:
    case TestPage::Count:
        break;
    }
}

void change_page(KeyEvent event, AppState& state)
{
    if (event != KeyEvent::Up && event != KeyEvent::Down) return;

    const int count = static_cast<int>(TestPage::Count);
    int next = static_cast<int>(state.page);
    if (event == KeyEvent::Up) next = (next + count - 1) % count;
    if (event == KeyEvent::Down) next = (next + 1) % count;
    state.page = static_cast<TestPage>(next);
}

void draw_screen(Hardware& hw, const AppState& state)
{
    TFTSPI_dir_cls(u16BLACK);
    drawf(0, u16GREEN, "[%u/%u] %s",
          page_index(state.page) + 1,
          static_cast<unsigned>(TestPage::Count),
          page_title(state.page));

    switch (state.page) {
    case TestPage::Buzzer:
        drawf(1, u16WHITE, "duration:%lums", static_cast<unsigned long>(state.buzzer_ms));
        draw_line(2, "OK: beep");
        draw_line(3, "UP/DN: page");
        break;

    case TestPage::Encoder:
        drawf(1, u16WHITE, "L:%8.2f", state.enc_l);
        drawf(2, u16WHITE, "R:%8.2f", state.enc_r);
        draw_line(3, "OK: reset counter");
        break;

    case TestPage::MotorPwm:
        drawf(1, state.motor_enabled ? u16GREEN : u16YELLOW,
              "state:%s", state.motor_enabled ? "ON" : "OFF");
        drawf(2, u16WHITE, "L duty:%d", state.motor_left_duty);
        drawf(3, u16WHITE, "R duty:%d", state.motor_right_duty);
        draw_line(4, "OK:on/off UP/DN");
        draw_line(5, "BACK/CAN: stop");
        break;

    case TestPage::EscPwm:
        drawf(1, state.esc_enabled ? u16GREEN : u16YELLOW,
              "state:%s", state.esc_enabled ? "ON" : "OFF");
        drawf(2, u16WHITE, "percent:%d%%", state.esc_percent);
        drawf(3, u16WHITE, "pwm:%d", static_cast<int>(ESC_MIN_PWM * (1.0f + state.esc_percent * 0.01f)));
        draw_line(4, "OK:on/off UP/DN");
        draw_line(5, "BACK/CAN: stop");
        break;

    case TestPage::Keys:
        drawf(1, u16WHITE, "UP:%d DN:%d", !hw.key_up->raw_level(), !hw.key_down->raw_level());
        drawf(2, u16WHITE, "OK:%d CA:%d", !hw.key_ok->raw_level(), !hw.key_cancel->raw_level());
        drawf(3, u16WHITE, "BACK:%d", !hw.key_back->raw_level());
        draw_line(4, "low level = press");
        break;

    case TestPage::Imu:
        drawf(1, imu_sys.is_initialized ? u16GREEN : u16RED,
              "init:%s", imu_sys.is_initialized ? "OK" : "FAIL");
        drawf(2, u16WHITE, "AX:%d AY:%d", imu_sys.raw_ax, imu_sys.raw_ay);
        drawf(3, u16WHITE, "AZ:%d", imu_sys.raw_az);
        drawf(4, u16WHITE, "GX:%d GY:%d", imu_sys.raw_gx, imu_sys.raw_gy);
        drawf(5, u16WHITE, "GZ:%d T:%.1f", imu_sys.raw_gz, imu_sys.temperature);
        break;

    case TestPage::Count:
        break;
    }

    draw_line(7, "UP/DN page OK test", u16CYAN);
    TFTSPI_dir_flush();
}

void init_hardware(Hardware& hw)
{
    try_load_required_modules();
    TFTSPI_dri_init(1);
    TFTSPI_dir_cls(u16BLACK);
    draw_line(0, "Blindbox tester", u16GREEN);
    draw_line(1, "initializing...");
    TFTSPI_dir_flush();

    buzzer_sys.init();
    imu_sys.init("/dev/lq_i2c_mpu6050");

    hw.enc_l.reset(new LS_PwmEncoder(0, 72));
    hw.enc_r.reset(new LS_PwmEncoder(1, 73));

    hw.motor_r_pwm.reset(new AtimPwm(81, 1, LS_ATIM_INVERSED, 17000, 0));
    hw.motor_l_pwm.reset(new AtimPwm(82, 2, LS_ATIM_INVERSED, 17000, 0));
    hw.motor_r_dir.reset(new HWGpio(21, GPIO_Mode_Out));
    hw.motor_l_dir.reset(new HWGpio(22, GPIO_Mode_Out));
    hw.motor_r_pwm->Enable();
    hw.motor_l_pwm->Enable();
    hw.motor_r_pwm->SetDutyCycle(0);
    hw.motor_l_pwm->SetDutyCycle(0);

    hw.esc_r_pwm.reset(new GtimPwm(89, 3, LS_GTIM_INVERSED, 50, ESC_MIN_PWM));
    hw.esc_l_pwm.reset(new GtimPwm(77, 4, LS_GTIM_INVERSED, 50, ESC_MIN_PWM, 0b01));
    hw.esc_r_pwm->Enable();
    hw.esc_l_pwm->Enable();
    hw.esc_r_pwm->SetDutyCycle(ESC_MIN_PWM);
    hw.esc_l_pwm->SetDutyCycle(ESC_MIN_PWM);

    hw.key_up.reset(new DebouncedKey("UP", KEY_UP_PIN, KeyEvent::Up));
    hw.key_down.reset(new DebouncedKey("DOWN", KEY_DOWN_PIN, KeyEvent::Down));
    hw.key_ok.reset(new DebouncedKey("OK", KEY_OK_PIN, KeyEvent::OK));
    hw.key_cancel.reset(new DebouncedKey("CANCEL", KEY_CANCEL_PIN, KeyEvent::Cancel));
    hw.key_back.reset(new DebouncedKey("BACK", KEY_BACK_PIN, KeyEvent::Back));
}

void safe_shutdown(Hardware& hw, AppState& state)
{
    motor_stop(hw, state);
    esc_stop(hw, state);
    buzzer_sys.off();
}

} // namespace

BayWatcher_Buzzer buzzer_sys;
BayWatcher_IMU imu_sys;

int main()
{
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    Hardware hw;
    AppState state;

    init_hardware(hw);

    auto last_draw = std::chrono::steady_clock::now() - std::chrono::milliseconds(200);
    while (g_running) {
        buzzer_sys.Tick();

        if (hw.enc_l && hw.enc_r) {
            state.enc_l = hw.enc_l->Update();
            state.enc_r = hw.enc_r->Update();
        }

        if (imu_sys.is_initialized) {
            imu_sys.update();
        }

        const KeyEvent event = scan_keys(hw);
        const TestPage page_before_event = state.page;
        if (state.page == TestPage::Buzzer || state.page == TestPage::Encoder ||
            state.page == TestPage::MotorPwm || state.page == TestPage::EscPwm) {
            handle_event(event, hw, state);
        } else if (event == KeyEvent::Cancel || event == KeyEvent::Back) {
            safe_shutdown(hw, state);
        }

        // 非 PWM 输出页：UP/DOWN 翻页。PWM 输出页：UP/DOWN 调输出。
        if ((event == KeyEvent::Up || event == KeyEvent::Down) &&
            page_before_event != TestPage::MotorPwm &&
            page_before_event != TestPage::EscPwm) {
            change_page(event, state);
        }

        // Motor/ESC 页用 BACK/CANCEL 停止输出，并切到下一项测试。
        if ((event == KeyEvent::Cancel || event == KeyEvent::Back) &&
            (page_before_event == TestPage::MotorPwm || page_before_event == TestPage::EscPwm)) {
            change_page(KeyEvent::Down, state);
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_draw > std::chrono::milliseconds(150) || event != KeyEvent::None) {
            draw_screen(hw, state);
            last_draw = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    safe_shutdown(hw, state);
    TFTSPI_dir_cls(u16BLACK);
    draw_line(0, "tester stopped", u16YELLOW);
    TFTSPI_dir_flush();
    return 0;
}
