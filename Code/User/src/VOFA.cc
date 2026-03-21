#include "main.hpp" 

BayWatcher_VOFA::BayWatcher_VOFA() {
    current_mode = Vofa_Mode::NONE;
    uart_cli = nullptr;
}

BayWatcher_VOFA::~BayWatcher_VOFA() {
    if (uart_cli != nullptr) {
        delete uart_cli;
    }
}

// ======================= 初始化 UDP =======================
void BayWatcher_VOFA::init_UDP(const std::string& target_ip, uint16_t target_port, uint16_t local_port) {
    if (is_initialized) return;
    
    // 如果底层的 init 成功
    if (udp_cli.UDP_client_init(target_ip, target_port) == 0) {
        int sock_fd = udp_cli.UDP_Get_socket();
        if (sock_fd >= 0) {
            // 设置非阻塞
            int flags = fcntl(sock_fd, F_GETFL, 0);
            fcntl(sock_fd, F_SETFL, flags | O_NONBLOCK);

            // 开启端口复用 (防止程序重启时端口被占用导致 bind 失败)
            int opt = 1;
            setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            // 3. 绑定本地端口
            struct sockaddr_in local_addr;
            memset(&local_addr, 0, sizeof(local_addr));
            local_addr.sin_family = AF_INET;
            local_addr.sin_port = htons(local_port); 
            local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
            
            // 【查 bind 的返回值！
            if (bind(sock_fd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
                // 如果这里打印了 Error，说明 UDP_client_init 真的把 socket 搞脏了
                perror("[VOFA UDP] Bind Local Port Failed"); 

            } else {
                printf("[VOFA UDP] Successfully bound to port: %d\n", local_port);
            }
            
            current_mode = Vofa_Mode::UDP;
            is_initialized = true;
        }
    }
}

// ======================= 初始化 TCP =======================
void BayWatcher_VOFA::init_TCP(const std::string& target_ip, uint16_t target_port) {
    if (is_initialized) return;
    if (tcp_cli.TCP_client_init(target_ip, target_port) == 0) {
        int sock_fd = tcp_cli.TCP_Get_socket();
        if (sock_fd >= 0) {
            // 设置非阻塞，防止 recv 卡死线程
            int flags = fcntl(sock_fd, F_GETFL, 0);
            fcntl(sock_fd, F_SETFL, flags | O_NONBLOCK);
            
            current_mode = Vofa_Mode::TCP;
            is_initialized = true;
        }
    }
}

// ======================= 初始化 UART =======================
void BayWatcher_VOFA::init_UART(const std::string& dev, speed_t baudrate) {
    if (is_initialized) return;
    if (uart_cli != nullptr) delete uart_cli;
    
    // 初始化并配置串口
    uart_cli = new LS_UART(dev, baudrate, LS_UART_STOP1, LS_UART_DATA8, LS_UART_NONE);
    current_mode = Vofa_Mode::UART;
    is_initialized = true;
}

// ======================= 发送数据 =======================
void BayWatcher_VOFA::Send_Data(float target_L, float speed_L, int pwm_L, 
                                float target_R, float speed_R, int pwm_R, 
                                float yaw, float base_speed,
                                float average_curvature, float preview_img_y) {
// void BayWatcher_VOFA::Send_Data(float base_speed, float speed_L, int pwm_L, 
//                                 float speed_R, int pwm_R, float speed_average,
//                                 float yaw, float current_yawspeed) {
    if (!is_initialized) return;

    char buf[256]; 
    memset(buf, 0, sizeof(buf)); 
    int len = sprintf(buf, "%.2f,%.2f,%d,%.2f,%.2f,%d,%.2f,%.2f,%.4f,%.2f\n", 
                      target_L, speed_L, pwm_L, target_R, speed_R, pwm_R,
                      yaw, base_speed, average_curvature, preview_img_y);
    // int len = sprintf(buf, "%.2f,%.2f,%d,%.2f,%d,%.2f,%.2f,%.2f\n", 
    //                   base_speed, speed_L, pwm_L, speed_R, pwm_R, speed_average, yaw,current_yawspeed);
    
    if (len > 0) {
        // 自动路由到对应的通信方式
        switch (current_mode) {
            case Vofa_Mode::UDP:  udp_cli.UDP_Send(buf, len); break;
            case Vofa_Mode::TCP:  tcp_cli.TCP_Send(buf, len); break;
            case Vofa_Mode::UART: uart_cli->WriteData(buf, len); break;
            default: break;
        }
    }
}

// ======================= 接收指令 =======================
void BayWatcher_VOFA::Process_Incoming(void) {
    if (!is_initialized) return;

    char buf[128];
    memset(buf, 0, sizeof(buf));
    ssize_t n = -1;
    
    // 非阻塞读取指令
//     switch (current_mode) {
//         case Vofa_Mode::UDP:  n = udp_cli.UDP_Recv(buf, sizeof(buf) - 1); break;
//         case Vofa_Mode::TCP:  n = tcp_cli.TCP_Recv(buf, sizeof(buf) - 1); break;
//         case Vofa_Mode::UART: n = uart_cli->ReadData(buf, sizeof(buf) - 1); break;
//         default: return;
//     }
    
//     if (n > 0) {
//         buf[n] = '\0'; 
        
//         if (strncmp(buf, "stop", 4) == 0) {
//             BayWatcher_Stop_Car();
//             BayWatcher_Menu::getInstance().refresh_menu(KeyOp::Up);
//         }
//         else if (strncmp(buf, "spd:", 4) == 0) {
//             BayWatcher_Set_BaseSpeed(atof(buf + 4));
//             BayWatcher_Menu::getInstance().refresh_menu(KeyOp::Up);
//         }
//         // 外环模糊 PID 调参
//         else if (strncmp(buf, "bkp:", 4) == 0) KP_Base = atof(buf + 4);
//         else if (strncmp(buf, "bkd:", 4) == 0) KD_Base = atof(buf + 4);
//         else if (strncmp(buf, "fkp:", 4) == 0) KP_Fuzzy = atof(buf + 4);
//         else if (strncmp(buf, "fkd:", 4) == 0) KD_Fuzzy = atof(buf + 4);
//         else if (strncmp(buf, "err:", 4) == 0) ERROR_MAX = atof(buf + 4);
//         // 内环调参
//         else if (strncmp(buf, "lkp:", 4) == 0) PID_Speed_L.Kp = atof(buf + 4);
//         else if (strncmp(buf, "lki:", 4) == 0) PID_Speed_L.Ki = atof(buf + 4);
//         else if (strncmp(buf, "lkd:", 4) == 0) PID_Speed_L.Kd = atof(buf + 4);
//         else if (strncmp(buf, "rkp:", 4) == 0) PID_Speed_R.Kp = atof(buf + 4);
//         else if (strncmp(buf, "rki:", 4) == 0) PID_Speed_R.Ki = atof(buf + 4);
//         else if (strncmp(buf, "rkd:", 4) == 0) PID_Speed_R.Kd = atof(buf + 4);
//     }
// }

switch (current_mode) {
        case Vofa_Mode::UDP:  
        {
            int sock_fd = udp_cli.UDP_Get_socket();
            n = recvfrom(sock_fd, buf, sizeof(buf) - 1, 0, NULL, NULL);
            if (n < 0) {
                // 如果是正常的非阻塞等待，直接清零 n 并跳过，绝不报错
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    n = 0; 
                } else {
                    perror("[VOFA UDP Error]"); // 只有真正的错误才打印
                }
            }
            break;
        }
        case Vofa_Mode::TCP:  
        {
            int sock_fd = tcp_cli.TCP_Get_socket();
            n = recv(sock_fd, buf, sizeof(buf) - 1, 0);
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) n = 0;
            break;
        }
        case Vofa_Mode::UART: 
            n = uart_cli->ReadData(buf, sizeof(buf) - 1); 
            break;
        default: return;
    }
    
    // 如果收到了有效数据，开始解析
    if (n > 0) {
        buf[n] = '\0'; 
        if (strncmp(buf, "start", 5) == 0) {
            BayWatcher_Start_Car();
            BayWatcher_Menu::getInstance().refresh_menu(KeyOp::Up);
        }
        else if (strncmp(buf, "stop", 4) == 0) {
            BayWatcher_Stop_Car();
            BayWatcher_Menu::getInstance().refresh_menu(KeyOp::Up);
        }
        else if (strncmp(buf, "spd:", 4) == 0) {
            BayWatcher_Set_BaseSpeed(atof(buf + 4));
            BayWatcher_Menu::getInstance().refresh_menu(KeyOp::Up);
        }
        // 外环模糊 PID 调参
        // else if (strncmp(buf, "bkp:", 4) == 0) KP_Base = atof(buf + 4);
        // else if (strncmp(buf, "bkd:", 4) == 0) KD_Base = atof(buf + 4);
        // else if (strncmp(buf, "fkp:", 4) == 0) KP_Fuzzy = atof(buf + 4);
        // else if (strncmp(buf, "fkd:", 4) == 0) KD_Fuzzy = atof(buf + 4);
        // else if (strncmp(buf, "err:", 4) == 0) ERROR_MAX = atof(buf + 4);
        // else if (strncmp(buf, "akp:", 4) == 0) PID_Angle.Kp = atof(buf + 4);
        // else if (strncmp(buf, "aki:", 4) == 0) PID_Angle.Ki = atof(buf + 4);
        // else if (strncmp(buf, "akd:", 4) == 0) PID_Angle.Kd = atof(buf + 4);
        // else if (strncmp(buf, "askp:", 4) == 0) PID_YawSpeed.Kp = atof(buf + 4);
        // else if (strncmp(buf, "aski:", 4) == 0) PID_YawSpeed.Ki = atof(buf + 4);
        // else if (strncmp(buf, "askd:", 4) == 0) PID_YawSpeed.Kd = atof(buf + 4);
        // 内环调参
        // else if (strncmp(buf, "lkp:", 4) == 0) PID_Speed_F_L.Kp = atof(buf + 4);
        // else if (strncmp(buf, "lki:", 4) == 0) PID_Speed_F_L.Ki = atof(buf + 4);
        // else if (strncmp(buf, "lkd:", 4) == 0) PID_Speed_F_L.Kd = atof(buf + 4);
        // else if (strncmp(buf, "rkp:", 4) == 0) PID_Speed_F_R.Kp = atof(buf + 4);
        // else if (strncmp(buf, "rki:", 4) == 0) PID_Speed_F_R.Ki = atof(buf + 4);
        // else if (strncmp(buf, "rkd:", 4) == 0) PID_Speed_F_R.Kd = atof(buf + 4);
        else if (strncmp(buf, "ckpa:", 4) == 0) PID_Cube.Kp_a = atof(buf + 4);
        else if (strncmp(buf, "ckdb:", 4) == 0) PID_Cube.Kp_b = atof(buf + 4);
        else if (strncmp(buf, "cki:", 4) == 0) PID_Cube.Ki = atof(buf + 4);
        else if (strncmp(buf, "ckda:", 4) == 0) PID_Cube.Kd_a = atof(buf + 4);
        else if (strncmp(buf, "ckdb:", 4) == 0) PID_Cube.Kd_b = atof(buf + 4);
        else if (strncmp(buf, "lkp:", 4) == 0) PID_Speed_L.Kp = atof(buf + 4);
        else if (strncmp(buf, "lki:", 4) == 0) PID_Speed_L.Ki = atof(buf + 4);
        else if (strncmp(buf, "lkd:", 4) == 0) PID_Speed_L.Kd = atof(buf + 4);
        else if (strncmp(buf, "rkp:", 4) == 0) PID_Speed_R.Kp = atof(buf + 4);
        else if (strncmp(buf, "rki:", 4) == 0) PID_Speed_R.Ki = atof(buf + 4);
        else if (strncmp(buf, "rkd:", 4) == 0) PID_Speed_R.Kd = atof(buf + 4);
    }
}
