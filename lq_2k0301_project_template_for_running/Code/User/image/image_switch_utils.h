#pragma once

#include <algorithm>
#include <cctype>
#include <string>

// 功能: 统一解析 on/off 型布尔文本。
// 类型: 图像侧公共工具函数。
// 关键参数: text-待解析文本, out_value-输出布尔值。
// 说明:
// - 识别链和图传链都复用这套解析规则，避免两个文件各维护一份字符串表。
// - 支持 1/0、on/off、true/false、yes/no、enable/disable 等常见写法。
static inline bool ParseImageBoolSwitchText(const std::string& text, bool* out_value)
{
    if (out_value == nullptr)
    {
        return false;
    }

    std::string normalized;
    normalized.resize(text.size());
    std::transform(text.begin(), text.end(), normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (normalized == "1" || normalized == "on" || normalized == "true" ||
        normalized == "yes" || normalized == "enable" || normalized == "enabled")
    {
        *out_value = true;
        return true;
    }

    if (normalized == "0" || normalized == "off" || normalized == "false" ||
        normalized == "no" || normalized == "disable" || normalized == "disabled")
    {
        *out_value = false;
        return true;
    }

    return false;
}
