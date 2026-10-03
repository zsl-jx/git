#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// 传感器、驱动、通信等硬件接口由后续代码补充实现。
// 这里只构建平衡车控制主程序框架。

#define CONTROL_LOOP_HZ 200
#define CONTROL_LOOP_DT (1.0f / CONTROL_LOOP_HZ)
#define MAX_TILT_ANGLE_DEG 20.0f
#define MOTOR_OUTPUT_LIMIT 1000

typedef struct {
    float angle;           // 当前姿态角（度）
    float angular_rate;    // 角速度（度/s）
} AttitudeState;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
} PIDController;

// 硬件接口占位符
bool hardware_init(void);
void imu_init(void);
void motor_init(void);
void imu_update(void);
void imu_get_attitude(AttitudeState *state);
void motor_set_output(int16_t left, int16_t right);
void delay_ms(uint32_t ms);
bool is_emergency_stop(void);

static float clampf(float x, float min_val, float max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

static int16_t clamp_i16(int32_t x, int16_t min_val, int16_t max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return (int16_t)x;
}

static float pid_compute(PIDController *pid, float target, float measurement)
{
    float error = target - measurement;
    pid->integral += error * CONTROL_LOOP_DT;
    float derivative = (error - pid->prev_error) / CONTROL_LOOP_DT;
    pid->prev_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = clampf(output, pid->output_min, pid->output_max);
    return output;
}

int main(void)
{
    AttitudeState attitude = {0};
    PIDController balance_pid = {
        .kp = 40.0f,
        .ki = 1.0f,
        .kd = 0.8f,
        .integral = 0.0f,
        .prev_error = 0.0f,
        .output_min = -MOTOR_OUTPUT_LIMIT,
        .output_max = MOTOR_OUTPUT_LIMIT,
    };

    // 初始化硬件和控制子系统
    if (!hardware_init()) {
        printf("Hardware init failed\n");
        return -1;
    }

    imu_init();
    motor_init();

    printf("Balance car main program started\n");

    while (1) {
        if (is_emergency_stop()) {
            motor_set_output(0, 0);
            printf("Emergency stop triggered\n");
            break;
        }

        imu_update();
        imu_get_attitude(&attitude);

        // 稳态控制：目标角度为 0 度，保持直立。
        float target_angle = 0.0f;

        if (attitude.angle > MAX_TILT_ANGLE_DEG || attitude.angle < -MAX_TILT_ANGLE_DEG) {
            motor_set_output(0, 0);
            printf("Tilt limit exceeded: %.2f deg\n", attitude.angle);
            break;
        }

        float control_output = pid_compute(&balance_pid, target_angle, attitude.angle);
        int16_t speed = clamp_i16((int32_t)control_output, -MOTOR_OUTPUT_LIMIT, MOTOR_OUTPUT_LIMIT);

        // 左右轮相同输出，后续可改为基于转向命令差动驱动
        motor_set_output(speed, speed);

        delay_ms((uint32_t)(CONTROL_LOOP_DT * 1000));
    }

    motor_set_output(0, 0);
    return 0;
}
//测试代码结束
//怎么登陆
