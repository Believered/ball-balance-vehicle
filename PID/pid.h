#ifndef __PID_H
#define __PID_H

/* 方向环在线调试参数；模式一直接修改实际控制参数。 */
extern float place_Kp;
extern float place_Ki;
extern float place_Kd;

/* 左右轮独立速度PID参数；模式二直接在线修改。 */
extern float VKp_l;
extern float VKi_l;
extern float VKd_l;
extern float VKp_r;
extern float VKi_r;
extern float VKd_r;

/* 模式一独立参数；模式二和模式五调参不会改动这些值。 */
extern float mode1_VKp_l;
extern float mode1_VKi_l;
extern float mode1_VKd_l;
extern float mode1_VKp_r;
extern float mode1_VKi_r;
extern float mode1_VKd_r;
extern float mode1_place_Kp;
extern float mode1_place_Ki;
extern float mode1_place_Kd;

float velocity_PID_value_l(float measure,float calcu);
float velocity_PID_value_r(float measure,float calcu);

/*
 * 低冲击任务专用速度PI：参数、积分限幅和采样周期与普通速度环完全相同，
 * 唯一差别是不叠加静态前馈。模式一/模式五使用它，模式二仍保留原调参路径。
 */
float velocity_PID_value_l_no_ff(float measure, float calcu);
float velocity_PID_value_r_no_ff(float measure, float calcu);
float PID_Mode1VelocityLeftNoFeedforward(float target, float measured);
float PID_Mode1VelocityRightNoFeedforward(float target, float measured);

/** 根据实车稳态数据分别生成左右轮基础前馈PWM。 */
float PID_VelocityFeedforwardLeft(float target);
float PID_VelocityFeedforwardRight(float target);
float place_PID_value(float measure,float calcu);
float place_PID_value_limited(float measure,
                              float calcu,
                              float output_limit);
float PID_Mode1PlaceLimited(float measure,
                            float calcu,
                            float output_limit);
/**
 * 将位置修正与左右轮实测速度差组成阻尼项。正修正=降低左轮，负修正=降低右轮。
 * 该函数不会在位置修正为0时自行产生转向，也不会让阻尼把修正方向翻转。
 */
float PID_ApplyPlaceRateDamping(float correction,
                                float measured_left,
                                float measured_right,
                                float output_limit);
float PID_Mode1ApplyPlaceRateDamping(float correction,
                                     float measured_left,
                                     float measured_right,
                                     float output_limit);
float angle_PID_value(float measure,float calcu);

/** 清除左右速度 PI 的积分和历史误差。 */
void PID_ResetVelocity(void);

/** 单独清除某一轮速度PI，用于循迹内轮目标降到0后的无扰再启动。 */
void PID_ResetVelocityLeft(void);
void PID_ResetVelocityRight(void);
void PID_ResetMode1VelocityLeft(void);
void PID_ResetMode1VelocityRight(void);
void PID_ResetMode1Velocity(void);

/**
 * 按左右轮各自当前目标、实测速度和上一拍PWM反算PI状态，实现无扰切换。
 */
void PID_PrimeVelocity(float target_left,
                       float target_right,
                       float measured_left,
                       float measured_right,
                       float output_left,
                       float output_right);

/* 按无前馈模型反算积分，用于PWM斜坡限速时的无扰跟踪。 */
void PID_PrimeVelocityNoFeedforward(float target_left,
                                    float target_right,
                                    float measured_left,
                                    float measured_right,
                                    float output_left,
                                    float output_right);
/** PWM斜率限制只影响单轮时，必须只播种对应轮，禁止左右PI相互干扰。 */
void PID_PrimeVelocityLeftNoFeedforward(float target,
                                        float measured,
                                        float output);
void PID_PrimeVelocityRightNoFeedforward(float target,
                                         float measured,
                                         float output);
void PID_PrimeMode1VelocityNoFeedforward(float target_left,
                                         float target_right,
                                         float measured_left,
                                         float measured_right,
                                         float output_left,
                                         float output_right);
/** 模式一独立速度PI的单轮无扰播种接口。 */
void PID_PrimeMode1VelocityLeftNoFeedforward(float target,
                                             float measured,
                                             float output);
void PID_PrimeMode1VelocityRightNoFeedforward(float target,
                                              float measured,
                                              float output);

/** 清除循迹位置控制器的中心迟滞、条件积分和历史状态。 */
void PID_ResetPlace(void);
void PID_ResetMode1Place(void);

/** 清除本工程当前使用的全部控制器状态。 */
void PID_ResetAll(void);

#endif
