#include "LQ_module_loader.hpp"
#include <iostream>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <array>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <sys/stat.h>

// 构造函数
LQModuleLoader::LQModuleLoader(const std::unordered_map<std::string, std::string>& modules)
    : modules_(modules) {
    
    // 设置加载顺序（先加载驱动，再加载设备）
    // 根据你的需求调整顺序，有依赖关系的模块需要先加载
    loadOrder_ = {"lq_i2c_all_dev", "lq_i2c_mpu6050_drv", "TFT18_dri", "TFT18_dev"};
    
    // 卸载顺序与加载顺序相反
    unloadOrder_ = {"TFT18_dev", "TFT18_dri", "lq_i2c_mpu6050_drv", "lq_i2c_all_dev"};
    
    // 初始化状态
    for (const auto& module : modules_) {
        moduleStatus_[module.first] = false;
    }
    
    // 设置模块路径（这里使用用户主目录）
    const char* homeDir = getenv("HOME");
    if (homeDir) {
        modulePath_ = std::string(homeDir) + "/";
    }
}

// 执行shell命令
std::string LQModuleLoader::executeCommand(const std::string& command) {
    std::array<char, 128> buffer;
    std::string result;
    
    // 使用popen执行命令
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    
    return result;
}

// 检查模块是否在lsmod输出中
bool LQModuleLoader::checkModuleInLsmod(const std::string& moduleBaseName) {
    std::string command = "lsmod | grep " + moduleBaseName;
    std::string result = executeCommand(command);
    
    // 如果结果包含模块基本名，说明模块已加载
    return result.find(moduleBaseName) != std::string::npos;
}

// 从完整路径中提取模块基本名
std::string LQModuleLoader::extractModuleBaseName(const std::string& fullPath) {
    std::string fileName;
    
    // 查找最后一个斜杠
    size_t lastSlash = fullPath.find_last_of("/");
    if (lastSlash != std::string::npos) {
        fileName = fullPath.substr(lastSlash + 1);
    } else {
        fileName = fullPath;
    }
    
    // 去除.ko后缀
    size_t koPos = fileName.find(".ko");
    if (koPos != std::string::npos) {
        fileName = fileName.substr(0, koPos);
    }
    
    return fileName;
}

// 从模块文件名中提取基本名
std::string LQModuleLoader::getModuleBaseName(const std::string& fileName) {
    std::string baseName = fileName;
    
    // 去除.ko后缀
    size_t koPos = baseName.find(".ko");
    if (koPos != std::string::npos) {
        baseName = baseName.substr(0, koPos);
    }
    
    return baseName;
}

// 检查模块是否已加载
bool LQModuleLoader::isModuleLoaded(const std::string& moduleName) {
    auto it = modules_.find(moduleName);
    if (it == modules_.end()) {
        std::cerr << "Module '" << moduleName << "' not found in module list!" << std::endl;
        return false;
    }
    
    std::string moduleBaseName = getModuleBaseName(it->second);
    return checkModuleInLsmod(moduleBaseName);
}

// 加载单个模块
bool LQModuleLoader::loadModule(const std::string& moduleName) {
    auto it = modules_.find(moduleName);
    if (it == modules_.end()) {
        std::cerr << "Module '" << moduleName << "' not found in module list!" << std::endl;
        return false;
    }
    
    std::string moduleFile = it->second;
    std::string modulePath = modulePath_ + moduleFile;
    std::string moduleBaseName = getModuleBaseName(moduleFile);
    
    std::cout << "Loading module: " << moduleName 
              << " (" << moduleFile << ")" << std::endl;
    
    // 检查模块文件是否存在
    struct stat buffer;
    std::string fullPath = modulePath_;
    if (modulePath_[0] == '~') {
        const char* homeDir = getenv("HOME");
        if (homeDir) {
            fullPath = std::string(homeDir) + modulePath_.substr(1);
        }
    }
    fullPath += moduleFile;
    
    if (stat(fullPath.c_str(), &buffer) != 0) {
        std::cerr << "Module file not found: " << fullPath << std::endl;
        return false;
    }
    
    // 检查模块是否已加载
    if (checkModuleInLsmod(moduleBaseName)) {
        std::cout << "Module " << moduleBaseName << " is already loaded. Unloading first..." << std::endl;
        
        // 先卸载模块
        std::string unloadCmd = "rmmod " + moduleBaseName + " 2>/dev/null";
        int unloadResult = system(unloadCmd.c_str());
        
        if (unloadResult != 0) {
            std::cerr << "Failed to unload module " << moduleBaseName << std::endl;
            return false;
        }
        
        std::cout << "Module " << moduleBaseName << " unloaded successfully." << std::endl;
    }
    
    // 加载模块
    std::string loadCmd = "insmod " + fullPath;
    int loadResult = system(loadCmd.c_str());
    
    if (loadResult != 0) {
        std::cerr << "Failed to load module " << moduleName << std::endl;
        std::cerr << "Command: " << loadCmd << std::endl;
        
        // 检查是否有依赖问题
        std::string dmesgCmd = "dmesg | tail -20";
        std::string dmesgOutput = executeCommand(dmesgCmd);
        std::cerr << "Last kernel messages:\n" << dmesgOutput << std::endl;
        
        moduleStatus_[moduleName] = false;
        return false;
    }
    
    // 验证模块是否成功加载
    if (checkModuleInLsmod(moduleBaseName)) {
        std::cout << "Module " << moduleName << " loaded successfully." << std::endl;
        moduleStatus_[moduleName] = true;
        return true;
    } else {
        std::cerr << "Module " << moduleName << " appears not to be loaded (check lsmod)." << std::endl;
        moduleStatus_[moduleName] = false;
        return false;
    }
}

// 卸载单个模块
bool LQModuleLoader::unloadModule(const std::string& moduleName) {
    auto it = modules_.find(moduleName);
    if (it == modules_.end()) {
        std::cerr << "Module '" << moduleName << "' not found in module list!" << std::endl;
        return false;
    }
    
    std::string moduleFile = it->second;
    std::string moduleBaseName = getModuleBaseName(moduleFile);
    
    std::cout << "Unloading module: " << moduleName << std::endl;
    
    // 检查模块是否已加载
    if (!checkModuleInLsmod(moduleBaseName)) {
        std::cout << "Module " << moduleBaseName << " is not loaded." << std::endl;
        moduleStatus_[moduleName] = false;
        return true;
    }
    
    // 卸载模块
    std::string unloadCmd = "rmmod " + moduleBaseName;
    int unloadResult = system(unloadCmd.c_str());
    
    if (unloadResult != 0) {
        std::cerr << "Failed to unload module " << moduleName << std::endl;
        std::cerr << "Command: " << unloadCmd << std::endl;
        
        // 检查是否有模块在使用
        std::string lsmodCmd = "lsmod | grep " + moduleBaseName;
        std::string lsmodOutput = executeCommand(lsmodCmd);
        std::cerr << "Current module status:\n" << lsmodOutput << std::endl;
        
        return false;
    }
    
    // 验证模块是否成功卸载
    if (!checkModuleInLsmod(moduleBaseName)) {
        std::cout << "Module " << moduleName << " unloaded successfully." << std::endl;
        moduleStatus_[moduleName] = false;
        return true;
    } else {
        std::cerr << "Module " << moduleName << " still appears to be loaded." << std::endl;
        return false;
    }
}

// 加载所有模块（按指定顺序）
bool LQModuleLoader::loadAllModules() {
    std::cout << "Loading all modules..." << std::endl;
    bool allSuccess = true;
    
    for (const auto& moduleName : loadOrder_) {
        auto it = modules_.find(moduleName);
        if (it != modules_.end()) {
            if (!loadModule(moduleName)) {
                std::cerr << "Failed to load module: " << moduleName << std::endl;
                allSuccess = false;
                // 可以选择继续加载其他模块或立即返回
                // 这里我们继续加载，但记录失败
            }
        }
    }
    
    if (allSuccess) {
        std::cout << "All modules loaded successfully." << std::endl;
    } else {
        std::cerr << "Some modules failed to load." << std::endl;
    }
    
    return allSuccess;
}

// 卸载所有模块（按指定顺序）
bool LQModuleLoader::unloadAllModules() {
    std::cout << "Unloading all modules..." << std::endl;
    bool allSuccess = true;
    
    for (const auto& moduleName : unloadOrder_) {
        auto it = modules_.find(moduleName);
        if (it != modules_.end()) {
            if (!unloadModule(moduleName)) {
                std::cerr << "Failed to unload module: " << moduleName << std::endl;
                allSuccess = false;
                // 继续尝试卸载其他模块
            }
        }
    }
    
    if (allSuccess) {
        std::cout << "All modules unloaded successfully." << std::endl;
    } else {
        std::cerr << "Some modules failed to unload." << std::endl;
    }
    
    return allSuccess;
}

// 获取模块加载状态
std::unordered_map<std::string, bool> LQModuleLoader::getModuleStatus() {
    // 更新状态
    for (auto& status : moduleStatus_) {
        status.second = isModuleLoaded(status.first);
    }
    return moduleStatus_;
}