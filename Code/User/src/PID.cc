#include "main.hpp"

// 增量式 PID 算法
static float Calc_Inc_PID(Bay_IncPID_t *pid, float target, float measured) {
    pid->error = target - measured;
    float inc = (pid->Kp * (pid->error-pid->prev_error)) +
                (pid->Ki * (pid->error)) +
                (pid->Kd * (pid->error - 2 * pid->prev_error + pid->last_error));
    pid->output += inc;
    // //增量限幅
    // float max_inc = 500.0f; 
    // if (inc > max_inc) inc = max_inc;
    // if (inc < -max_inc) inc = -max_inc;
    // PWM 限幅
    if (pid->output > pid->output_limit) pid->output = pid->output_limit;
    if (pid->output < -pid->output_limit) pid->output = -pid->output_limit;
    pid->last_error = pid->prev_error;
    pid->prev_error = pid->error;
    return pid->output;
}

// 位置式 PID 算法
static float Calc_Pos_PID(Bay_PosPID_t *pid, float target, float measured) {
    pid->error = target - measured;
    pid->integral += pid->error;
    if(pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float output = (pid->Kp * pid->error) + 
                   (pid->Ki * pid->integral) + 
                   (pid->Kd * (pid->error - pid->last_error));
    pid->last_error = pid->error;
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
    return output;
}

void BayWatcher_Control_Init(void) {
    memset(&PID,           0, sizeof(Bay_PID_t));
    memset(&PID_Speed_L,   0, sizeof(Bay_IncPID_t));
    memset(&PID_Speed_R,   0, sizeof(Bay_IncPID_t));
    memset(&PID_Speed_F_L, 0, sizeof(Bay_PosPID_t));
    memset(&PID_Speed_F_R, 0, sizeof(Bay_PosPID_t));
    memset(&PID_Angle,     0, sizeof(Bay_PosPID_t)); // 最外环
    memset(&PID_YawSpeed,  0, sizeof(Bay_PosPID_t)); // 中间环

    // 参数初始化
    PID.is_running = 0;         // 默认停止
    PID.base_target_speed = 0;  // 基础速度 0
    PID.target_angle = 0;       // 目标角度 0 (走直线)
    PID.target_yaw_speed = 0;       // 目标角度 0 (走直线)
    vL = 0;
    vR = 0;

    // // // 左轮速度环PID
    // // PID_Speed_L.Kp = 8.18f; PID_Speed_L.Ki = 3.90f; PID_Speed_L.Kd = 0.04f;
    // // PID_Speed_L.output = 0; PID_Speed_L.output_limit = 4550.0f;

    // // // 右轮速度环PID
    // // PID_Speed_R.Kp = 8.18f; PID_Speed_R.Ki = 3.96f; PID_Speed_R.Kd = 0.04f;
    // // PID_Speed_R.output = 0; PID_Speed_R.output_limit = 4550.0f;
    
    // // // 角速度环PID
    // // PID_YawSpeed.Kp = 0.00f;  
    // // PID_YawSpeed.Ki = 0.0f;   // 角速度环一般不需要 I，容易导致滞后
    // // PID_YawSpeed.Kd = 0.0f;  // 抑制震荡   
    // // PID_YawSpeed.output_limit = 40.0f;  PID_YawSpeed.integral_limit = 100.0f;
    
    // // // 角度环PID
    // // PID_Angle.Kp = 0.0f;     
    // // PID_Angle.Ki = 0.0f;      // 角度环也不建议给 I，除非走直线总是歪
    // // PID_Angle.Kd = 0.0f;      // 抑制震荡
    // // PID_Angle.output_limit = 360.0f; // 最大允许角速度 (度/秒)
    // // PID_Angle.integral_limit = 100.0f;

    // // 左轮速度环PID
    // PID_Speed_L.Kp = 35.07f; PID_Speed_L.Ki = 3.72f; PID_Speed_L.Kd = 0.00f;
    // PID_Speed_L.output = 0; PID_Speed_L.output_limit = 6000.0f;

    // // 右轮速度环PID
    // PID_Speed_R.Kp = 35.04f; PID_Speed_R.Ki = 3.92f; PID_Speed_R.Kd = 0.00f;
    // PID_Speed_R.output = 0; PID_Speed_R.output_limit = 6000.0f;

    // // // 左轮前进环PID
    // // PID_Speed_F_L.Kp = 0.00f; PID_Speed_F_L.Ki = 0.00f; PID_Speed_F_L.Kd = 0.00f;
    // // PID_Speed_F_L.integral_limit= 0; PID_Speed_F_L.output_limit = 6000;

    // // // 左轮前进环PID
    // // PID_Speed_F_R.Kp = 0.00f; PID_Speed_F_R.Ki = 0.00f; PID_Speed_F_R.Kd = 0.00f;
    // // PID_Speed_F_R.integral_limit= 0; PID_Speed_F_R.output_limit = 6000;
    
    // // 模糊PID
    // KP_Base = 0.088f; KD_Base = 3.621f; ERROR_MAX = 40.0f; // 设置模糊控制的范围
    // PID_Angle.Kp=KP_Base ; PID_Angle.Kd = KD_Base;
    // KD_Fuzzy = 0.0f; KP_Fuzzy = 0.0f; // 设置模糊调节的力度
    // 左轮速度环PID
    PID_Speed_L.Kp = 105.32f; PID_Speed_L.Ki = 7.15f; PID_Speed_L.Kd = 0.00f;
    PID_Speed_L.output = 0; PID_Speed_L.output_limit = 6000.0f;

    // 右轮速度环PID
    PID_Speed_R.Kp = 105.00f; PID_Speed_R.Ki = 7.06f; PID_Speed_R.Kd = 0.00f;
    PID_Speed_R.output = 0; PID_Speed_R.output_limit = 6000.0f;
    
    //模糊PID(外环)
    KP_Base = 0.088f;   
    KD_Base = 3.621f; 
    // KD_Base = KP_Base * 5;   
    PID_Angle.Kp = 0.0f;  
    PID_Angle.Ki = 0.0f; 
    PID_Angle.Kd = 0.0f;   
    PID_Angle.output_limit =  360.0f; 
    PID_Angle.integral_limit = 100.0f;
    ERROR_MAX = 40.0f; // 设置模糊控制的范围
    KP_Fuzzy = 0.0f; // 设置模糊调节的力度
    KD_Fuzzy = 0.0f;
}

#define MOTOR_DEAD_ZONE 500 
static int32_t Add_Dead_Zone(int32_t pwm) {
    int32_t abs_pwm = abs(pwm);
    int32_t compensation = 0;
    if (abs_pwm < 200) return 0;
    if (abs_pwm <= 1000) {
        compensation = MOTOR_DEAD_ZONE;
    }
    // else if (abs_pwm < 1500) {
    //     float ratio = (1500.0f - abs_pwm) / 500.0f;
    //     compensation = (int32_t)(MOTOR_DEAD_ZONE * ratio);
    // }
    // else {
    //     compensation = 0;
    // }
    if (pwm > 0) return pwm + compensation;
    if (pwm < 0) return pwm - compensation;
    return pwm;
}


void BayWatcher_Middle_Loop(void* arg){

}
void BayWatcher_Outter_Loop(void* arg){
    if (PID.is_running == 0) {
        PID_Speed_L.output = 0; PID_Speed_L.prev_error = 0;
        PID_Speed_R.output = 0; PID_Speed_R.prev_error = 0;
        PID.pwm_out_L = 0; PID.pwm_out_R = 0;
        motor_sys.Stop();
        return;
    }
    PID.vision_yaw = pure_angle;
    // get_updataPD_pid(PID.current_angle, &PID_Angle.Kp, &PID_Angle.Kd);
    PID.target_yaw_speed = Calc_Pos_PID(&PID_Angle, PID.target_angle, PID.vision_yaw);//右偏为-，左偏为+
    if(PID.target_yaw_speed > 25.0f)  PID.target_yaw_speed = 25.0f;
    if(PID.target_yaw_speed < -25.0f) PID.target_yaw_speed = -25.0f;
//     PID.speed_adjust = Calc_Pos_PID(&PID_Angle, PID.target_angle, PID.vision_yaw);//右偏为-，左偏为+
//     if(PID.speed_adjust > 25.0f) PID.speed_adjust = 25.0f;
//     if(PID.speed_adjust < -25.0f) PID.speed_adjust = -25.0f;
}

void BayWatcher_Inner_Loop(void* arg){
    if (PID.is_running == 0) {
        PID_Speed_L.output = 0; PID_Speed_L.prev_error = 0;
        PID_Speed_R.output = 0; PID_Speed_R.prev_error = 0;
        PID.pwm_out_L = 0; PID.pwm_out_R = 0;
        motor_sys.Stop();
        return;
    }
    // imu_sys.update();
    PID.speed_adjust = Calc_Pos_PID(&PID_YawSpeed, PID.target_yaw_speed, imu_sys.raw_gz);
    if(PID.speed_adjust > 1000.0f) PID.speed_adjust = 1000.0f;
    if(PID.speed_adjust < -1000.0f) PID.speed_adjust = -1000.0f;

    PID.speed_adjust = 0;
    vL = encoder_sys.getSpeedL();
    vR = encoder_sys.getSpeedR();
    v_avg = (vL + vR)*0.5 ;
    
    if(vL>=-0.01 && vL<=0.01) vL =0.0f;
    if(vR>=-0.01 && vR<=0.01) vR =0.0f;

    int32_t pid_out_L = (int32_t)Calc_Pos_PID(&PID_Speed_F_L, PID.base_target_speed, v_avg);
    int32_t pid_out_R = (int32_t)Calc_Pos_PID(&PID_Speed_F_R, PID.base_target_speed, v_avg);

    PID.pwm_out_L = pid_out_L+PID.speed_adjust;  
    PID.pwm_out_R = pid_out_R-PID.speed_adjust;

    // PID.pwm_out_L = 1000;
    // PID.pwm_out_R = 1000;

    motor_sys.Motor2_Set(PID.pwm_out_L);
    motor_sys.Motor1_Set(PID.pwm_out_R);

}


void BayWatcher_Set_BaseSpeed(float speed)   {PID.base_target_speed = speed;}
void BayWatcher_Set_TargetAngle(float angle) {PID.target_angle      = angle; }

void BayWatcher_Start_Car(void) {
    vL = 0; // 重置滤波状态
    vR=  0;
    // 手动启动：解除斑马线锁停
    // zebra_stop = 0;
    PID.is_running = 1;
    // Logger.Log_Event("START_LOGGING");
    PID_Speed_L.prev_error = 0; PID_Speed_L.last_error = 0; PID_Speed_L.output = 0;
    PID_Speed_R.prev_error = 0; PID_Speed_R.last_error = 0; PID_Speed_R.output = 0;
    PID_Angle.integral = 0; PID_Angle.last_error = 0;
    PID_Speed_F_L.integral = 0; PID_Speed_F_L.last_error = 0; PID_Speed_F_L.output = 0;
    PID_Speed_F_R.integral = 0; PID_Speed_F_R.last_error = 0; PID_Speed_F_R.output = 0;
    PID.base_target_speed = 0;
}

void BayWatcher_Stop_Car(void) {
    PID.is_running = 0;
    PID.base_target_speed = 0;
    motor_sys.Stop();
}

void BayWatcher_Control_Loop(void* arg) {

    // //Plan A:三串级PID
    // // 内部静态计数器，用于分频
    // static uint32_t sys_tick = 0;
    // sys_tick++;

    // // //视觉请求停车（斑马线等）：直接停电机并复位控制器，避免转向环继续输出导致自旋。
    // // if (zebra_stop)
    // // {
    // //     PID_Speed_L.output = 0; PID_Speed_L.prev_error = 0; PID_Speed_L.last_error = 0;
    // //     PID_Speed_R.output = 0; PID_Speed_R.prev_error = 0; PID_Speed_R.last_error = 0;
    // //     PID_Angle.integral = 0; PID_Angle.last_error = 0;
    // //     PID_YawSpeed.integral = 0; PID_YawSpeed.last_error = 0;

    // //     Robot.target_yaw_speed = 0;
    // //     Robot.speed_adjust = 0;
    // //     Robot.target_speed_L = 0;
    // //     Robot.target_speed_R = 0;
    // //     Robot.pwm_out_L = 0;
    // //     Robot.pwm_out_R = 0;
    // //     Motors.Stop();
    // //     return;
    // // }

    // if (PID.is_running == 0) {
    //     PID_Speed_L.output = 0; PID_Speed_L.prev_error = 0;
    //     PID_Speed_R.output = 0; PID_Speed_R.prev_error = 0;
    //     Robot.pwm_out_L = 0; Robot.pwm_out_R = 0;
    //     Motors.Stop();
    //     return;
    // }

    // // ================== 10ms 任务：位置环 (外环) ==================
    // if (sys_tick % 10 == 0) {
    //     PID.vision_yaw = pure_angle;
    //     // printf("%.1f\n",Robot.vision_yaw);
    //     Robot.current_angle = Robot.vision_yaw; 
    //     Robot.target_yaw_speed = Calc_Pos_PID(&PID_Angle, Robot.target_angle, Robot.current_angle);//右偏为-，左偏为+
    //     // printf("%.1f\n",Robot.target_yaw_speed);

    //     // //介入模糊PID
    //     // // 计算误差: 目标(0) - 当前角度
    //     // float current_error = Robot.target_angle - Robot.current_angle;//极性
    //     // get_updataPD_pid(current_error, &PID_Angle.Kp, &PID_Angle.Kd);
    //     // if(PID_Angle.Kp < 0) PID_Angle.Kp = 0;
    //     // if(PID_Angle.Kd < 0) PID_Angle.Kd = 0;
    //     // Robot.target_yaw_speed = Calc_Pos_PID(&PID_Angle, Robot.target_angle, Robot.current_angle);
    // }

    // // ================== 5ms 任务：位置环 (外环) ==================
    // if (sys_tick % 5 == 0) {
    //     Robot.current_yaw_speed = BayWatcher_Get_GyroZ_dps(); //gyroz右转为+左转为-低速:+-10~20;高速:+-50~100）
    //     // printf("%.1f\n",Robot.current_yaw_speed);
    //     Robot.speed_adjust = Calc_Pos_PID(&PID_YawSpeed, Robot.target_yaw_speed, Robot.current_yaw_speed);
    //     // printf("%.1f\n",Robot.speed_adjust);
    //     Robot.target_speed_L = Robot.base_target_speed + Robot.speed_adjust;
    //     Robot.target_speed_R = Robot.base_target_speed - Robot.speed_adjust;
    //     // printf("%.1f\n",Robot.target_speed_L);
    //     // printf("%.1f\n",Robot.target_speed_R);
        
    //     // 限幅
    //     if(Robot.target_speed_L > 60.0f) Robot.target_speed_L = 60.0f;
    //     if(Robot.target_speed_L < -60.0f) Robot.target_speed_L = -60.0f;
    //     if(Robot.target_speed_R > 60.0f) Robot.target_speed_R = 60.0f;
    //     if(Robot.target_speed_R < -60.0f) Robot.target_speed_R = -60.0f;
    // }

    // // ================== 2ms 任务：速度环 (内环) ==================
    // if (sys_tick % 2 == 0) {
    //     // 读取速度 (2ms周期)
    //     Robot.enc_speed_L = Encoders.Get_Speed_L_cm_s();
    //     Robot.enc_speed_R = Encoders.Get_Speed_R_cm_s();

    //     Robot.alpha = 0.7f;

    //     Robot.speed_L_filter = Robot.alpha * (float)Robot.enc_speed_L + (1.0f - Robot.alpha) * Robot.speed_L_filter;
    //     Robot.speed_R_filter = Robot.alpha * (float)Robot.enc_speed_R + (1.0f - Robot.alpha) * Robot.speed_R_filter;
        
    //     Robot.enc_speed_L = Robot.speed_L_filter;
    //     Robot.enc_speed_R = Robot.speed_R_filter;

    //     // Robot.target_speed_L = 5.0f; // 假装有个目标，方便看波形
    //     // Robot.target_speed_R = 5.0f;
        
    //     // Robot.pwm_out_L = test_pwm;   // 左轮给正
    //     // Robot.pwm_out_R = test_pwm;   // 右轮给正

    //     // // 3. 输出给电机
    //     // Motors.Motor1_Set(Robot.pwm_out_L); 
    //     // Motors.Motor2_Set(Robot.pwm_out_R); 

    //     // // Robot.pwm_out_L = 1000;  
    //     // // Robot.pwm_out_R = 1000;

    //     // 计算 PID
    //     int32_t pid_out_L = (int32_t)Calc_Inc_PID(&PID_Speed_L, Robot.target_speed_L, Robot.enc_speed_L);
    //     int32_t pid_out_R = (int32_t)Calc_Inc_PID(&PID_Speed_R, Robot.target_speed_R, Robot.enc_speed_R);

    //     // // 智能死区补偿
    //     // Robot.pwm_out_L = Add_Dead_Zone(pid_out_L);
    //     // Robot.pwm_out_R = Add_Dead_Zone(pid_out_R);

    //     Robot.pwm_out_L = pid_out_L;  
    //     Robot.pwm_out_R = pid_out_R;

    //     // 输出
    //     Motors.Motor1_Set(Robot.pwm_out_L);
    //     Motors.Motor2_Set(Robot.pwm_out_R);
    // }

    //Plan B：模糊PID双串
    if (PID.is_running == 0) {
        PID_Speed_L.output = 0; PID_Speed_L.prev_error = 0;
        PID_Speed_R.output = 0; PID_Speed_R.prev_error = 0;
        PID.pwm_out_L = 0; PID.pwm_out_R = 0;
        motor_sys.Stop();
        return;
    }
    float diff_speed = 0.0f;

    static uint32_t sys_tick = 0;
    sys_tick++;
    // ================== 10ms 任务：位置环 (模糊) ==================
    if (sys_tick % 2 == 0) {
        // if (handler_sys.Is_Executing()) {
        //     // [盲走阶段]：剥夺摄像头的控制权，使用固定的差速跑动作
        //     diff_speed = handler_sys.Get_Differential_Override();
        //     PID.speed_adjust = diff_speed;
            
        // } else {
            // 把控制权还给摄像头
            PID.vision_yaw = pure_angle;
            // printf("%.1f\n",PID.vision_yaw);
            PID.current_angle = PID.vision_yaw; 
            // PID.current_angle = 0;
            get_updataPD_pid(PID.current_angle, &PID_Angle.Kp, &PID_Angle.Kd);
            PID.speed_adjust = Calc_Pos_PID(&PID_Angle, PID.target_angle, PID.current_angle);//右偏为-，左偏为+
            // printf("%.1f\n",PID.target_yaw_speed);
            if(PID.speed_adjust > 25.0f) PID.speed_adjust = 25.0f;
            if(PID.speed_adjust < -25.0f) PID.speed_adjust = -25.0f;
        // }
        // PID.vision_yaw = pure_angle;
        // // printf("%.1f\n",PID.vision_yaw);
        // PID.current_angle = PID.vision_yaw; 
        // // PID.current_angle = 0;
        // get_updataPD_pid(PID.current_angle, &PID_Angle.Kp, &PID_Angle.Kd);
        // PID.speed_adjust = Calc_Pos_PID(&PID_Angle, PID.target_angle, PID.current_angle);//右偏为-，左偏为+
        // // printf("%.1f\n",PID.target_yaw_speed);
        // if(PID.speed_adjust > 25.0f) PID.speed_adjust = 25.0f;
        // if(PID.speed_adjust < -25.0f) PID.speed_adjust = -25.0f;
    }
    // ================== 5ms 任务：速度环 (内环) ==================
        // 读取速度 (5ms周期)
        vL = encoder_sys.getSpeedL();
        vR = encoder_sys.getSpeedR();
        if(vL>=-0.01 && vL<=0.01) vL =0.0f;
        if(vR>=-0.01 && vR<=0.01) vR =0.0f;

        // // PID.pwm_out_L = 1000;  
        // // PID.pwm_out_R = 1000;

        PID.target_speed_L=PID.base_target_speed+PID.speed_adjust;
        PID.target_speed_R=PID.base_target_speed-PID.speed_adjust;

        // 计算 PID
        int32_t pid_out_L = (int32_t)Calc_Inc_PID(&PID_Speed_L, PID.target_speed_L, vL);
        int32_t pid_out_R = (int32_t)Calc_Inc_PID(&PID_Speed_R, PID.target_speed_R, vR);

        // // 智能死区补偿
        // PID.pwm_out_L = Add_Dead_Zone(pid_out_L);
        // PID.pwm_out_R = Add_Dead_Zone(pid_out_R);

        PID.pwm_out_L = pid_out_L;  
        PID.pwm_out_R = pid_out_R;

        // PID.pwm_out_L = 1000;  
        // PID.pwm_out_R = 1000;

        // 输出
        motor_sys.Motor2_Set(PID.pwm_out_L);
        motor_sys.Motor1_Set(PID.pwm_out_R);
}

// /**
//  * @brief 带前馈补偿的位置式PID控制器
//  * @param target    目标设定值（如目标位置、温度）
//  * @param actual    当前实际测量值
//  * @param Kp        比例系数
//  * @param Ki        积分系数
//  * @param Kff       前馈增益（根据系统模型或实验调整）
//  * @return          控制量输出（如PWM占空比、电机电压）
//  */
// float Feedforward_PID(float target, float actual, float Kp, float Ki, float Kff)
// {
//     // 静态变量声明（保存历史状态）
//     static float error_sum = 0;   // 积分项累积
//     static float last_error = 0;  // 上一次误差
//     static float last_target = 0; // 上一次目标值（用于前馈计算）
 
//     // 计算当前误差
//     float error = target - actual;
 
//     // PID计算
//     error_sum += error; // 积分项累积
//     float pid_output = Kp * error + Ki * error_sum;
 
//     // 前馈计算
//     float feedforward = Kff * (target - last_target);
 
//     // 总控制量 = PID输出 + 前馈补偿
//     float output = pid_output + feedforward;
 
//     // 更新历史数据
//     last_error = error;
//     last_target = target;
 
//     return output;
// }