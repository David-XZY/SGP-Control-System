#ifndef __CONTROL_MANAGER_H
#define __CONTROL_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

/* 受控轴数量 */
#define CTRL_AXIS_NUM 6

/* 系统主状态机定义 */
typedef enum {
    SYS_BOOT = 0,          /* 上电初始化阶段 */
    SYS_IDLE,              /* 空闲停车 */
    SYS_RUN_CLOSED_LOOP,   /* 闭环运行 */
    SYS_HOMING,            /* 并行回零 */
    SYS_MANUAL_TEST,       /* 手动测试输出 */
    SYS_TUNE_VEL,          /* 单轴速度环调参 */
    SYS_TUNE_POS,          /* 单轴位置环调参 */
    SYS_SYNC_TEST,         /* 6 轴同步控制测试 */
    SYS_ESTOP_LATCHED,     /* 急停锁存 */
    SYS_FAULT              /* 故障 */
} SystemMode_e;

/**
 * @brief 初始化控制管理器
 * @param 无
 * @return 无
 */
void ControlMgr_Init(void);

/**
 * @brief 10ms 周期任务（建议在 TIM6 中断调用）
 * @param 无
 * @return 无
 */
void ControlMgr_Tick10ms(void);

/**
 * @brief 主循环后台任务（状态上报等）
 * @param 无
 * @return 无
 */
void ControlMgr_MainTask(void);

/**
 * @brief 切换到闭环运行模式
 * @param 无
 * @return 无
 */
void ControlMgr_SetRunMode(void);

/**
 * @brief 切换到空闲模式
 * @param 无
 * @return 无
 */
void ControlMgr_SetIdleMode(void);

/**
 * @brief 启动并行回零
 * @param 无
 * @return 无
 */
void ControlMgr_StartHoming(void);

/**
 * @brief 中止回零
 * @param 无
 * @return 无
 */
void ControlMgr_AbortHoming(void);

/**
 * @brief 软件复位系统
 * @param 无
 * @return 无
 */
void ControlMgr_RequestReset(void);

/**
 * @brief 设置 6 轴目标位置数组
 * @param pos [in] 长度为 6 的目标位置数组（单位 mm）
 * @return 无
 */
void ControlMgr_SetTargetPosAll(const float pos[CTRL_AXIS_NUM]);

/**
 * @brief 设置单轴目标位置
 * @param axis [in] 轴号（0~5）
 * @param pos  [in] 目标位置（单位 mm）
 * @return 无
 */
void ControlMgr_SetTargetPosAxis(uint8_t axis, float pos);

/**
 * @brief 设置单轴 PID 与速度限幅参数
 * @param axis      [in] 轴号（0~5）
 * @param kp_pos    [in] 位置环 P
 * @param kp_vel    [in] 速度环 P
 * @param ki_vel    [in] 速度环 I
 * @param kd_vel    [in] 速度环 D
 * @param vel_limit [in] 速度限幅（绝对值，单位 mm/s）
 * @return 无
 */
void ControlMgr_SetPidAxis(uint8_t axis, float kp_pos, float kp_vel, float ki_vel, float kd_vel, float vel_limit);

/**
 * @brief 设置全部轴 PID 与速度限幅参数
 * @param kp_pos    [in] 位置环 P
 * @param kp_vel    [in] 速度环 P
 * @param ki_vel    [in] 速度环 I
 * @param kd_vel    [in] 速度环 D
 * @param vel_limit [in] 速度限幅（绝对值，单位 mm/s）
 * @return 无
 */
void ControlMgr_SetPidAll(float kp_pos, float kp_vel, float ki_vel, float kd_vel, float vel_limit);
void ControlMgr_EnterManualTest(void);
void ControlMgr_ExitManualTest(void);
void ControlMgr_ManualSetAxisOutput(uint8_t axis, uint8_t in1, uint8_t in2, uint16_t pwm);
void ControlMgr_ManualBrakeAll(void);
void ControlMgr_StartTuneVel(uint8_t axis, float target_vel);
void ControlMgr_StartTunePos(uint8_t axis, float target_pos);
void ControlMgr_StopTune(void);
uint8_t ControlMgr_GetTuneAxis(void);
bool ControlMgr_StartSyncHome40(void);
bool ControlMgr_StartSyncExtend(float start_len, float end_len, float speed);
bool ControlMgr_StartSyncSine(float center, float amp, float freq, float cycles);
void ControlMgr_StopSyncTest(void);
bool ControlMgr_IsSyncTestActive(void);
uint8_t ControlMgr_GetSyncState(void);
float ControlMgr_GetSyncRefLen(void);
float ControlMgr_GetSyncMaxErr(void);
float ControlMgr_GetSyncSpread(void);
float ControlMgr_GetAxisLenMm(uint8_t axis);

/**
 * @brief 获取当前系统模式
 * @param 无
 * @return SystemMode_e 当前状态
 */
SystemMode_e ControlMgr_GetMode(void);

/**
 * @brief 查询急停是否锁存
 * @param 无
 * @return true 已锁存，false 未锁存
 */
bool ControlMgr_IsEstopLatched(void);

#endif
