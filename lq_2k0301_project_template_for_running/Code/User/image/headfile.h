#ifndef _HEADFILE_H_
#define _HEADFILE_H_

/* 图像模块历史兼容聚合头（作用域: 全局包含聚合）
 * 说明：
 * - 主要服务于旧代码和上层快速接入。
 * - 新的核心实现文件应尽量改用最小必要 include，避免把系统库依赖继续扩散。
 */
/* 第三方库 */
#include <opencv2/opencv.hpp>
// #include <json/json.h>
// #include <net.h>

/* 系统库 */
#include <iostream>
#include <chrono>
#include <fstream>
#include <cmath>
#include <cstdint>
#include <thread>
#include <mutex>
#include <sstream>
#include <vector>
#include <ctime>
#include <iomanip>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/fb.h>
#include <sys/mman.h>
// #include <linux/input.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netdb.h>
#include <cstring>

#endif
