#include "main.hpp"
#include "recognition_chain.h"
#include "recognition_runtime.h"
#include "roi_capture_runtime.h"
#include "stream_chain.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

using namespace cv;

// ==================== 全局对象与外部声明 ====================
std::unique_ptr<lq_camera_ex> camera;
TransmissionStreamServer  server;
Mat                       img;
Mat                       view;

namespace {

struct StartupWhiteReferenceStats
{
    bool valid = false;
    int sample_count = 0;
    double mean_b = 0.0;
    double mean_g = 0.0;
    double mean_r = 0.0;
    double mean_luma = 0.0;
};

struct StartupLightingSample
{
    int exposure = 0;
    double gain = 0.0;
    double wb_blue = 0.0;
    double wb_red = 0.0;
    StartupWhiteReferenceStats white_stats;
};

struct CameraManualSettings
{
    bool use_manual_exposure = false;
    int exposure = BW_RECOG_CAMERA_MANUAL_EXPOSURE;
    bool use_manual_gain = false;
    double gain = BW_RECOG_CAMERA_MANUAL_GAIN;
    bool use_manual_wb = false;
    double wb_blue = BW_RECOG_CAMERA_MANUAL_WB_BLUE;
    double wb_red = BW_RECOG_CAMERA_MANUAL_WB_RED;
};

const char* camera_format_text(lq_camera_format_t format)
{
    switch (format)
    {
    case LQ_CAMERA_0CPU_MJPG: return "0CPU_MJPG";
    case LQ_CAMERA_HIGH_MJPG: return "HIGH_MJPG";
    default: return "UNKNOWN_FMT";
    }
}

int clamp_int(int value, int min_value, int max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

bool is_positive_finite(double value)
{
    return std::isfinite(value) && value > 0.0;
}

std::string get_executable_directory()
{
    char buffer[4096] = {0};
    const ssize_t len = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (len <= 0)
    {
        return ".";
    }
    buffer[len] = '\0';
    std::string full_path(buffer);
    const std::string::size_type pos = full_path.find_last_of("/\\");
    if (pos == std::string::npos)
    {
        return ".";
    }
    if (pos == 0)
    {
        return full_path.substr(0, 1);
    }
    return full_path.substr(0, pos);
}

std::string join_path(const std::string& base, const std::string& name)
{
    if (base.empty())
    {
        return name;
    }
    const char tail = base[base.size() - 1];
    if (tail == '/' || tail == '\\')
    {
        return base + name;
    }
    return base + "/" + name;
}

std::string get_startup_lighting_config_path()
{
    return join_path(get_executable_directory(),
                     BW_RECOG_STARTUP_FIVE_POINT_LIGHTING_CONFIG_FILE);
}

char read_blocking_command_char()
{
    for (;;)
    {
        const int ch = std::getchar();
        if (ch == EOF)
        {
            clearerr(stdin);
            usleep(10 * 1000);
            continue;
        }
        if (ch == '\n' || ch == '\r')
        {
            continue;
        }
        return static_cast<char>(ch);
    }
}

void flush_stdin_pending_input()
{
    tcflush(STDIN_FILENO, TCIFLUSH);
}

bool compute_startup_white_reference_stats(const cv::Mat& frame_bgr,
                                           StartupWhiteReferenceStats* out_stats)
{
    if (out_stats == nullptr)
    {
        return false;
    }
    *out_stats = StartupWhiteReferenceStats();
    if (frame_bgr.empty())
    {
        return false;
    }

    const int row_y = std::max(0, std::min(BW_RECOG_WHITE_REFERENCE_ROW_Y, frame_bgr.rows - 1));
    const int half_height = std::max(0, BW_RECOG_WHITE_REF_STATS_HALF_HEIGHT);
    const int y0 = std::max(0, row_y - half_height);
    const int y1 = std::min(frame_bgr.rows - 1, row_y + half_height);
    if (y1 < y0)
    {
        return false;
    }

    const cv::Mat roi_bgr = frame_bgr.rowRange(y0, y1 + 1);
    cv::Mat roi_hsv;
    cv::cvtColor(roi_bgr, roi_hsv, cv::COLOR_BGR2HSV);

    double sum_b = 0.0;
    double sum_g = 0.0;
    double sum_r = 0.0;
    double sum_luma = 0.0;
    int sample_count = 0;

    for (int y = 0; y < roi_bgr.rows; ++y)
    {
        const cv::Vec3b* bgr_row = roi_bgr.ptr<cv::Vec3b>(y);
        const cv::Vec3b* hsv_row = roi_hsv.ptr<cv::Vec3b>(y);
        for (int x = 0; x < roi_bgr.cols; ++x)
        {
            const cv::Vec3b bgr = bgr_row[x];
            const cv::Vec3b hsv = hsv_row[x];
            const int max_rgb = std::max(std::max(static_cast<int>(bgr[0]), static_cast<int>(bgr[1])),
                                         static_cast<int>(bgr[2]));
            const int min_rgb = std::min(std::min(static_cast<int>(bgr[0]), static_cast<int>(bgr[1])),
                                         static_cast<int>(bgr[2]));
            const bool is_seed_white =
                hsv[1] <= static_cast<unsigned char>(BW_RECOG_WHITE_REF_SEED_MAX_SATURATION) &&
                hsv[2] >= static_cast<unsigned char>(BW_RECOG_WHITE_REF_SEED_MIN_VALUE) &&
                min_rgb >= BW_RECOG_WHITE_REF_SEED_MIN_RGB &&
                (max_rgb - min_rgb) <= BW_RECOG_WHITE_REF_SEED_MAX_CHANNEL_DIFF;
            if (!is_seed_white)
            {
                continue;
            }

            const double b = static_cast<double>(bgr[0]);
            const double g = static_cast<double>(bgr[1]);
            const double r = static_cast<double>(bgr[2]);
            sum_b += b;
            sum_g += g;
            sum_r += r;
            sum_luma += 0.114 * b + 0.587 * g + 0.299 * r;
            ++sample_count;
        }
    }

    if (sample_count <= 0)
    {
        return false;
    }

    out_stats->valid = true;
    out_stats->sample_count = sample_count;
    out_stats->mean_b = sum_b / sample_count;
    out_stats->mean_g = sum_g / sample_count;
    out_stats->mean_r = sum_r / sample_count;
    out_stats->mean_luma = sum_luma / sample_count;
    return true;
}

CameraManualSettings build_configured_camera_manual_settings()
{
    CameraManualSettings settings;
    settings.use_manual_exposure = (BW_RECOG_CAMERA_USE_MANUAL_EXPOSURE != 0);
    settings.exposure = BW_RECOG_CAMERA_MANUAL_EXPOSURE;
    settings.use_manual_gain = (BW_RECOG_CAMERA_USE_MANUAL_GAIN != 0);
    settings.gain = BW_RECOG_CAMERA_MANUAL_GAIN;
    settings.use_manual_wb = (BW_RECOG_CAMERA_USE_MANUAL_WHITE_BALANCE != 0);
    settings.wb_blue = BW_RECOG_CAMERA_MANUAL_WB_BLUE;
    settings.wb_red = BW_RECOG_CAMERA_MANUAL_WB_RED;
    return settings;
}

bool apply_camera_manual_settings(const CameraManualSettings& settings,
                                  const char* log_prefix)
{
    if (!camera || !camera->is_cam_opened())
    {
        return false;
    }

    const char* prefix = (log_prefix != nullptr) ? log_prefix : "Camera";
    bool any_ok = false;

    if (settings.use_manual_exposure)
    {
        if (!camera->set_exposure_manual(static_cast<int16_t>(settings.exposure)))
        {
            printf("[%s] 手动曝光设置失败，当前仍沿用摄像头默认曝光模式。\n", prefix);
        }
        else
        {
            any_ok = true;
            printf("[%s] 手动曝光已设置为 %d。\n", prefix, settings.exposure);
        }
    }
    else if (BW_RECOG_TEXT_LOG_ENABLE != 0)
    {
        printf("[%s] 当前沿用摄像头默认自动曝光。\n", prefix);
    }

    if (settings.use_manual_gain)
    {
        if (!camera->set_gain_manual(settings.gain))
        {
            printf("[%s] 手动增益设置失败，当前仍沿用摄像头默认增益策略。\n", prefix);
        }
        else
        {
            any_ok = true;
            printf("[%s] 手动增益已设置为 %.2f。\n", prefix, settings.gain);
        }
    }
    else if (BW_RECOG_TEXT_LOG_ENABLE != 0)
    {
        printf("[%s] 当前沿用摄像头默认增益策略。\n", prefix);
    }

    if (settings.use_manual_wb)
    {
        if (!camera->set_white_balance_manual(settings.wb_blue, settings.wb_red))
        {
            printf("[%s] 手动白平衡设置失败，当前仍沿用摄像头默认白平衡策略。\n", prefix);
        }
        else
        {
            any_ok = true;
            printf("[%s] 手动白平衡已设置为 blue=%.2f red=%.2f。\n",
                   prefix,
                   settings.wb_blue,
                   settings.wb_red);
        }
    }
    else if (BW_RECOG_TEXT_LOG_ENABLE != 0)
    {
        printf("[%s] 当前沿用摄像头默认白平衡策略。\n", prefix);
    }

    return any_ok;
}

void print_current_camera_readback(const char* log_prefix)
{
    if (!camera || !camera->is_cam_opened())
    {
        return;
    }

    const char* prefix = (log_prefix != nullptr) ? log_prefix : "Camera";
    const double exposure = camera->get_exposure_value();
    const double gain = camera->get_gain_value();
    const double wb_blue = camera->get_white_balance_blue_u_value();
    const double wb_red = camera->get_white_balance_red_v_value();
    printf("[%s] 当前相机读回值：exposure=%.2f gain=%.2f wb_blue=%.2f wb_red=%.2f\n",
           prefix,
           exposure,
           gain,
           wb_blue,
           wb_red);
}

bool set_camera_auto_sampling_mode()
{
    if (!camera || !camera->is_cam_opened())
    {
        return false;
    }

    bool ok = false;
    ok = camera->set_exposure_auto() || ok;
    ok = camera->set_white_balance_auto() || ok;
    return ok;
}

cv::Mat capture_settled_frame(int settle_frames)
{
    cv::Mat frame;
    const int total_frames = std::max(1, settle_frames);
    for (int i = 0; i < total_frames; ++i)
    {
        frame = camera->get_frame_raw();
        if (frame.empty())
        {
            usleep(10 * 1000);
        }
    }
    return frame;
}

double median_double(std::vector<double> values, double fallback_value)
{
    if (values.empty())
    {
        return fallback_value;
    }
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

int median_int(std::vector<int> values, int fallback_value)
{
    if (values.empty())
    {
        return fallback_value;
    }
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

bool save_startup_lighting_config(const CameraManualSettings& settings)
{
    const std::string path = get_startup_lighting_config_path();
    std::ofstream fout(path.c_str(), std::ios::out | std::ios::trunc);
    if (!fout.is_open())
    {
        printf("[Lighting] 持久化配置写入失败：%s\n", path.c_str());
        return false;
    }

    fout << "exposure " << settings.exposure << "\n";
    fout << "gain " << settings.gain << "\n";
    fout << "wb_blue " << settings.wb_blue << "\n";
    fout << "wb_red " << settings.wb_red << "\n";
    fout.close();

    printf("[Lighting] 五点采光配置已保存到：%s\n", path.c_str());
    return true;
}

bool load_startup_lighting_config(CameraManualSettings* out_settings)
{
    if (out_settings == nullptr)
    {
        return false;
    }

    const std::string path = get_startup_lighting_config_path();
    std::ifstream fin(path.c_str());
    if (!fin.is_open())
    {
        printf("[Lighting] 未找到持久化配置文件：%s\n", path.c_str());
        return false;
    }

    CameraManualSettings settings = build_configured_camera_manual_settings();
    settings.use_manual_exposure = true;
    settings.use_manual_gain = true;
    settings.use_manual_wb = true;

    std::string key;
    double value = 0.0;
    bool has_exposure = false;
    bool has_gain = false;
    bool has_wb_blue = false;
    bool has_wb_red = false;
    while (fin >> key >> value)
    {
        if (key == "exposure")
        {
            settings.exposure = clamp_int(static_cast<int>(std::lround(value)), 1, 10000);
            has_exposure = true;
        }
        else if (key == "gain")
        {
            settings.gain = value;
            has_gain = is_positive_finite(value);
        }
        else if (key == "wb_blue")
        {
            settings.wb_blue = value;
            has_wb_blue = is_positive_finite(value);
        }
        else if (key == "wb_red")
        {
            settings.wb_red = value;
            has_wb_red = is_positive_finite(value);
        }
    }
    fin.close();

    if (!(has_exposure && has_gain && has_wb_blue && has_wb_red))
    {
        printf("[Lighting] 持久化配置文件内容不完整：%s\n", path.c_str());
        return false;
    }

    *out_settings = settings;
    printf("[Lighting] 已读取持久化配置：exposure=%d gain=%.2f wb_blue=%.2f wb_red=%.2f\n",
           settings.exposure,
           settings.gain,
           settings.wb_blue,
           settings.wb_red);
    return true;
}

bool run_startup_five_point_lighting(CameraManualSettings* out_settings)
{
    if (out_settings == nullptr || !camera || !camera->is_cam_opened())
    {
        return false;
    }

    printf("[Lighting] 启动五点采光模式。按 c 采当前点，按 q 读取已保存配置并跳过重采，按 s 跳过并直接进入正常流程。\n");
    printf("[Lighting] 持久化配置文件路径：%s\n", get_startup_lighting_config_path().c_str());
    flush_stdin_pending_input();

    std::vector<StartupLightingSample> samples;
    samples.reserve(BW_RECOG_STARTUP_FIVE_POINT_LIGHTING_POINT_COUNT);
    bool auto_sampling_mode_armed = false;

    while (static_cast<int>(samples.size()) < BW_RECOG_STARTUP_FIVE_POINT_LIGHTING_POINT_COUNT)
    {
        const int point_index = static_cast<int>(samples.size()) + 1;
        printf("[Lighting] 等待第 %d/%d 个采光点，按 c 开始采样，按 q 读取配置跳过。\n",
               point_index,
               BW_RECOG_STARTUP_FIVE_POINT_LIGHTING_POINT_COUNT);

        const char cmd = read_blocking_command_char();
        if (cmd == 'q' || cmd == 'Q')
        {
            if (load_startup_lighting_config(out_settings))
            {
                printf("[Lighting] 已使用持久化配置，跳过五点采光。\n");
                return true;
            }
            printf("[Lighting] 读取持久化配置失败，请继续五点采光或按 s 回退默认参数。\n");
            continue;
        }
        if (cmd == 's' || cmd == 'S')
        {
            printf("[Lighting] 已跳过五点采光，回退到预设固定参数。\n");
            return false;
        }
        if (cmd != 'c' && cmd != 'C')
        {
            continue;
        }

        if (!auto_sampling_mode_armed)
        {
            if (!set_camera_auto_sampling_mode())
            {
                printf("[Lighting] 自动曝光/自动白平衡切换失败，继续按当前相机状态采样。\n");
            }
            else
            {
                printf("[Lighting] 已切入自动采光模式，从第 1 个点开始采样。\n");
            }
            auto_sampling_mode_armed = true;
        }

        const cv::Mat frame =
            capture_settled_frame(BW_RECOG_STARTUP_FIVE_POINT_LIGHTING_SETTLE_FRAMES);
        if (frame.empty())
        {
            printf("[Lighting] 第 %d/%d 点取帧失败，请重试。\n",
                   point_index,
                   BW_RECOG_STARTUP_FIVE_POINT_LIGHTING_POINT_COUNT);
            continue;
        }

        StartupLightingSample sample;
        if (!compute_startup_white_reference_stats(frame, &sample.white_stats) ||
            !sample.white_stats.valid)
        {
            printf("[Lighting] 第 %d/%d 点白参考统计失败，请确保画面里有白赛道并重试。\n",
                   point_index,
                   BW_RECOG_STARTUP_FIVE_POINT_LIGHTING_POINT_COUNT);
            continue;
        }

        const double raw_exposure = camera->get_exposure_value();
        sample.exposure = is_positive_finite(raw_exposure)
                              ? clamp_int(static_cast<int>(std::lround(raw_exposure)), 1, 10000)
                              : 0;
        sample.gain = camera->get_gain_value();
        sample.wb_blue = camera->get_white_balance_blue_u_value();
        sample.wb_red = camera->get_white_balance_red_v_value();
        samples.push_back(sample);

        printf("[Lighting] point=%d exposure=%d gain=%.2f wb_blue=%.2f wb_red=%.2f white_luma=%.2f white_samples=%d\n",
               point_index,
               sample.exposure,
               sample.gain,
               sample.wb_blue,
               sample.wb_red,
               sample.white_stats.mean_luma,
               sample.white_stats.sample_count);
    }

    std::vector<int> exposures;
    std::vector<double> gains;
    std::vector<double> wb_blues;
    std::vector<double> wb_reds;
    for (size_t i = 0; i < samples.size(); ++i)
    {
        if (samples[i].exposure > 0)
        {
            exposures.push_back(samples[i].exposure);
        }
        if (is_positive_finite(samples[i].gain))
        {
            gains.push_back(samples[i].gain);
        }
        if (is_positive_finite(samples[i].wb_blue))
        {
            wb_blues.push_back(samples[i].wb_blue);
        }
        if (is_positive_finite(samples[i].wb_red))
        {
            wb_reds.push_back(samples[i].wb_red);
        }
    }

    CameraManualSettings calibrated = build_configured_camera_manual_settings();
    calibrated.use_manual_exposure = true;
    calibrated.use_manual_gain = true;
    calibrated.use_manual_wb = true;
    calibrated.exposure = median_int(exposures, BW_RECOG_CAMERA_MANUAL_EXPOSURE);
    calibrated.gain = median_double(gains, BW_RECOG_CAMERA_MANUAL_GAIN);
    calibrated.wb_blue = median_double(wb_blues, BW_RECOG_CAMERA_MANUAL_WB_BLUE);
    calibrated.wb_red = median_double(wb_reds, BW_RECOG_CAMERA_MANUAL_WB_RED);
    *out_settings = calibrated;

    printf("[Lighting] 五点采光完成，采用中位数固定参数：exposure=%d gain=%.2f wb_blue=%.2f wb_red=%.2f\n",
           calibrated.exposure,
           calibrated.gain,
           calibrated.wb_blue,
           calibrated.wb_red);
    save_startup_lighting_config(calibrated);
    return true;
}

// ==================== 系统初始化辅助函数 ====================
// 功能: 初始化识别板串口，并在首次失败后按固定节奏有限次重试
// 类型: 局部功能函数
// 关键参数: 无
// 说明：上电早期串口设备节点可能还没完全就绪；这里做有限重试，但不会无限阻塞识别板启动。
static bool init_board_comm_with_retry()
{
    if (comm.init(UART1, B115200))
    {
        return true;
    }

    for (int retry = 1; retry <= BW_BOARD_COMM_INIT_RETRY_TIMES; ++retry)
    {
        if (BW_RECOG_TEXT_LOG_ENABLE != 0)
        {
            printf("[BoardComm] reconnecting... attempt %d/%d in %d ms\n",
                   retry,
                   BW_BOARD_COMM_INIT_RETRY_TIMES,
                   BW_BOARD_COMM_INIT_RETRY_INTERVAL_MS);
        }
        usleep((useconds_t)BW_BOARD_COMM_INIT_RETRY_INTERVAL_MS * 1000u);

        if (comm.init(UART1, B115200))
        {
            if (BW_RECOG_TEXT_LOG_ENABLE != 0)
            {
                printf("[BoardComm] reconnect success on attempt %d/%d\n",
                       retry,
                       BW_BOARD_COMM_INIT_RETRY_TIMES);
            }
            return true;
        }
    }

    printf("[BoardComm] reconnect stopped after %d failed attempts\n",
           BW_BOARD_COMM_INIT_RETRY_TIMES);
    return false;
}

// 功能: 系统初始化
// 类型: 局部功能函数
// 关键参数: 无
// 说明：识别板不再承担巡线/控制职责，上电后固定进入“彩色采集 + 识别链 + UART 发包”。
void system_init()
{
    if (BW_RECOG_TEXT_LOG_ENABLE != 0)
    {
        printf("Initializing recognition board...\n");
    }
    lq_camera_format_t requested_format = LQ_CAMERA_HIGH_MJPG;
    auto create_camera = [](lq_camera_format_t format) -> std::unique_ptr<lq_camera_ex> {
        return std::unique_ptr<lq_camera_ex>(new lq_camera_ex(
            recognition_runtime::kRecognitionFrameWidth,
            recognition_runtime::kRecognitionFrameHeight,
            recognition_runtime::kRecognitionFrameFps,
            format));
    };

#if BW_RECOG_CAMERA_TRY_0CPU_MJPG
    requested_format = LQ_CAMERA_0CPU_MJPG;
    camera = create_camera(LQ_CAMERA_0CPU_MJPG);
    if (camera && camera->is_cam_opened())
    {
        if (BW_RECOG_TEXT_LOG_ENABLE != 0)
        {
            printf("[Camera] using low-cpu MJPG mode.\n");
        }
    }
    else
    {
        camera.reset();
        if (BW_RECOG_TEXT_LOG_ENABLE != 0)
        {
            printf("[Camera] low-cpu MJPG unavailable, fallback to high MJPG.\n");
        }
    }
#endif

    if (!camera)
    {
        requested_format = LQ_CAMERA_HIGH_MJPG;
        camera = create_camera(LQ_CAMERA_HIGH_MJPG);
        if (camera && camera->is_cam_opened())
        {
            if (BW_RECOG_TEXT_LOG_ENABLE != 0)
            {
                printf("[Camera] using high MJPG mode.\n");
            }
        }
    }
    if (!camera || !camera->is_cam_opened())
    {
        printf("[Camera] 识别板相机打开失败。\n");
    }
    else
    {
#if BW_RECOG_STARTUP_FIVE_POINT_LIGHTING_ENABLE
        printf("[Lighting] 已启用启动五点采光，暂缓写入固定曝光/增益/白平衡。\n");
#else
        printf("[Lighting] 启动采光已关闭，将自动读取上次保存的采光配置。\n");
#endif
    }
    if (camera && camera->is_cam_opened())
    {
        printf("[Camera] request=%ux%u@%u format=%s, actual_fps=%u\n",
               (unsigned)recognition_runtime::kRecognitionFrameWidth,
               (unsigned)recognition_runtime::kRecognitionFrameHeight,
               (unsigned)recognition_runtime::kRecognitionFrameFps,
               camera_format_text(requested_format),
               (unsigned)camera->get_camera_fps());
    }
    if (!init_board_comm_with_retry())
    {
        printf("[BoardComm] tx uart unavailable, recognition will continue without board link\n");
    }
    if (BW_RECOG_TEXT_LOG_ENABLE != 0)
    {
        printf("Recognition board init done.\n");
    }
}

} // namespace

// ==================== 主函数 ====================
int main(int argc, char** argv)
{
    // 系统初始化
    system_init();

    if (camera && camera->is_cam_opened())
    {
        CameraManualSettings startup_camera_settings;
#if BW_RECOG_STARTUP_FIVE_POINT_LIGHTING_ENABLE
        if (!run_startup_five_point_lighting(&startup_camera_settings))
        {
            startup_camera_settings = build_configured_camera_manual_settings();
            printf("[Lighting] 使用预设固定参数作为启动相机参数。\n");
        }
#else
        if (!load_startup_lighting_config(&startup_camera_settings))
        {
            startup_camera_settings = build_configured_camera_manual_settings();
            printf("[Lighting] 自动读取上次采光配置失败，回退预设固定参数。\n");
        }
        else
        {
            printf("[Lighting] 启动采光已关闭，已自动使用上次保存参数，无需按 q。\n");
        }
#endif
        apply_camera_manual_settings(startup_camera_settings, "Lighting");
        print_current_camera_readback("Lighting");
    }

    // 设置终端为非阻塞（用于按键 'c' 快速复位识别链状态）
    {
        const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags != -1)
        {
            fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        }
    }

    if (BW_RECOG_TEXT_LOG_ENABLE != 0)
    {
        std::cout << "输入 c: 启动一次红块检测与识别链" << std::endl;
    }

    // 识别板保留图传开关和识别开关两个入口
    const bool stream_enabled =
        StreamChain::ParseSwitch(argc, argv, StreamChain::DefaultEnabled());
    const bool recognition_enabled =
        RecognitionChain::ParseSwitch(argc, argv, RecognitionChain::DefaultEnabled());
    const bool roi_capture_enabled =
        ParseRoiCaptureModeSwitch(argc, argv, RoiCaptureModeDefaultEnabled());
    const RoiCaptureTransferConfig roi_capture_transfer_config =
        ParseRoiCaptureTransferConfig(argc, argv);

    if (BW_RECOG_TEXT_LOG_ENABLE != 0)
    {
        std::cout << "[BOOT] stream switch=" << (stream_enabled ? "on" : "off")
                  << " (args: --stream / --no-stream / --stream=on|off)" << std::endl;
        std::cout << "[BOOT] recognition switch=" << (recognition_enabled ? "on" : "off")
                  << " (args: --recognition / --no-recognition / --recognition=on|off)" << std::endl;
        std::cout << "[BOOT] roi capture switch=" << (roi_capture_enabled ? "on" : "off")
                  << " (args: --roi-capture / --no-roi-capture / --mode=roi|recognition)"
                  << std::endl;
        if (roi_capture_enabled)
        {
            std::cout << "[BOOT] roi capture target="
                      << (roi_capture_transfer_config.host.empty() ? "<unset>" : roi_capture_transfer_config.host)
                      << ":" << roi_capture_transfer_config.port
                      << " (args: --roi-host=<pc_ip> --roi-port=<port>)"
                      << std::endl;
        }
        std::cout << "[BOOT] recognition board camera="
                  << recognition_runtime::kRecognitionFrameWidth << "x"
                  << recognition_runtime::kRecognitionFrameHeight << "@"
                  << recognition_runtime::kRecognitionFrameFps
                  << " color, UART1@115200" << std::endl;
        std::cout << "[BOOT] loop_target_fps="
                  << BW_RECOG_LOOP_TARGET_FPS
                  << std::endl;
    }

    if (roi_capture_enabled)
    {
        RunRoiCaptureBoard(stream_enabled, roi_capture_transfer_config);
        return 0;
    }

    // 启动识别板独立运行时（阻塞运行）
    RunRecognitionBoard(stream_enabled, recognition_enabled);
    return 0;
}
