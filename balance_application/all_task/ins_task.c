#include "ins_task.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can_user.h"
#include "pid.h"
#include "cmsis_os2.h"
#include "MahonyAHRS.h"
#include "math.h"

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

/** 磁力计(IST8310)原始数据数组[3]=xyz。意义：历史遗留的磁力计数据缓存，工程实际未使用（本工程已改用 ist8310_real_data 结构体）。写：无（未使用）；读：无（未使用）。 */
fp32 ist8310_data[3] = {0,0,0};
/** 定时器 TIM10 句柄（CubeMX 定义于 main.c）。用于 BMI088 温度加热 PWM（CH1）。写：HAL 驱动内部/CubeMX；读：imu_pwm_set（__HAL_TIM_SetCompare）、Imu_Task（HAL_TIM_PWM_Start）。 */
extern TIM_HandleTypeDef htim10;

static void imu_pwm_set(uint16_t pwm);
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);
static void imu_temp_control(fp32 temp);
static void imu_cmd_spi_dma(void);
void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);


/** SPI1 外设句柄（CubeMX 定义于 main.c）。意义：SPI1 用于与 BMI088 陀螺仪/加速度计/温度传感器进行 DMA 全双工通信。写：Imu_Task（改 BaudRatePrescaler 并 HAL_SPI_Init）、SPI1_DMA_init（置 CR2 DMAEN）、HAL 驱动内部；读：imu_cmd_spi_dma（hdmatx/hdmarx 判断 DMA 忙）、SPI1_DMA_init/enable、HAL_SPI_TxRxCpltCallback（Instance）。 */
extern SPI_HandleTypeDef hspi1;
/** SPI1 接收方向 DMA 句柄（CubeMX 定义于 main.c，Stream2）。意义：SPI1 RX DMA 配置。写：HAL DMA 驱动/CubeMX；读：imu_cmd_spi_dma（经 hspi1.hdmarx 检查 DMA_SxCR_EN 判断忙闲）。 */
extern DMA_HandleTypeDef hdma_spi1_rx;
/** SPI1 发送方向 DMA 句柄（CubeMX 定义于 main.c）。意义：SPI1 TX DMA 配置。写：HAL DMA 驱动/CubeMX；读：imu_cmd_spi_dma（经 hspi1.hdmatx 检查 DMA_SxCR_EN 判断忙闲）。 */
extern DMA_HandleTypeDef hdma_spi1_tx;

/** 保存 Imu_Task 自身的线程 ID。意义：供 EXTI 软中断回调定位目标线程以便唤醒。写：Imu_Task（osThreadGetId() 赋值一次）；读：HAL_GPIO_EXTI_Callback（osThreadFlagsSet 唤醒）。 */
static osThreadId_t INS_task_local_handler;

/** 陀螺仪(BMI088) SPI DMA 接收缓冲。意义：保存一次 SPI 读回的陀螺仪原始寄存器数据帧。写：SPI 接收 DMA（硬件写入）；读：Imu_Task（BMI088_gyro_read_over 解析）。 */
uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
/** 陀螺仪 SPI DMA 发送缓冲（指令帧）。意义：首字节 0x82=陀螺仪数据读命令，其余 0xFF 为占位时钟。写：静态初值（编译期）；读：SPI 发送 DMA（作为 TX 源）。 */
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

/** 加速度计(BMI088) SPI DMA 接收缓冲。写：SPI 接收 DMA；读：Imu_Task（BMI088_accel_read_over 解析）。 */
uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
/** 加速度计 SPI DMA 发送缓冲（指令帧）。首字节 0x92=加速度计读命令。写：静态初值；读：SPI 发送 DMA。 */
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

/** 加速度计内置温度 SPI DMA 接收缓冲。写：SPI 接收 DMA；读：Imu_Task（BMI088_temperature_read_over 解析）。 */
uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
/** 温度 SPI DMA 发送缓冲（指令帧）。首字节 0xA2=温度读命令。写：静态初值；读：SPI 发送 DMA。 */
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};

/** 陀螺仪数据更新状态标志。意义：按位记录处理阶段——位(IMU_DR_SHFITS)=数据就绪、位(IMU_SPI_SHFITS)=DMA 传输中、位(IMU_UPDATE_SHFITS)=已读回待解算。写：HAL_GPIO_EXTI_Callback（置 DR）、HAL_SPI_TxRxCpltCallback（SPI→UPDATE）、Imu_Task（解算后清 UPDATE）；读：imu_cmd_spi_dma、HAL_SPI_TxRxCpltCallback、Imu_Task。 */
volatile uint8_t gyro_update_flag = 0;
/** 加速度计数据更新状态标志（含义同 gyro_update_flag）。写：EXTI 回调（置 DR）、TxRxCplt 回调（SPI→UPDATE）、Imu_Task（清 UPDATE）；读：imu_cmd_spi_dma、TxRxCplt 回调、Imu_Task。 */
volatile uint8_t accel_update_flag = 0;
/** 温度数据更新状态标志（含义同 gyro_update_flag）。写：EXTI 回调（置 DR）、TxRxCplt 回调（SPI→UPDATE）、Imu_Task（清 UPDATE）；读：imu_cmd_spi_dma、TxRxCplt 回调、Imu_Task。 */
volatile uint8_t accel_temp_update_flag = 0;
/** 磁力计数据更新状态标志（含义同 gyro_update_flag）。写：EXTI 回调（置 DR）、Imu_Task（DR→SPI 流转）；读：imu_cmd_spi_dma（未参与，本工程磁力计未接入 DMA）、Imu_Task。 */
volatile uint8_t mag_update_flag = 0;
/** SPI DMA 启动使能标志。意义：Imu_Task 完成 SPI1 DMA 初始化前置 1，之后 EXTI 才允许发起采样。写：Imu_Task（置 1）；读：HAL_GPIO_EXTI_Callback（判断是否发起 imu_cmd_spi_dma）。 */
volatile uint8_t imu_start_dma_flag = 0;

/** BMI088(陀螺仪+加速度计+温度) 原始数据结构。写：Imu_Task（BMI088_read / BMI088_*_read_over 填充 gyro/accel/temp/time）；读：Imu_Task（imu_cali_slove 取原始值）。 */
bmi088_real_data_t bmi088_real_data;
/** 陀螺仪安装旋转矩阵（把传感器轴系换算到机体坐标系）。写：静态初值（BMI088_BOARD_INSTALL_SPIN_MATRIX）；读：imu_cali_slove。 */
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
/** 陀螺仪零漂（单位 rad/s），imu_cali_slove 中加性补偿。写：INS_cali_gyro、INS_set_cali_gyro、gyro_offset_calc（迭代累减）；读：imu_cali_slove。 */
fp32 gyro_offset[3];
/** 陀螺仪标定零漂存档。意义：标定完成后保存，供下次上电/重新标定时加载。写：INS_set_cali_gyro；读：INS_cali_gyro（time_count==0 时加载）。 */
fp32 gyro_cali_offset[3];

/** 加速度计安装旋转矩阵。写：静态初值；读：imu_cali_slove。 */
fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
/** 加速度计零漂（单位 m/s2）。写：INS_set_cali_gyro 等外部标定（本文件未改，保留）；读：imu_cali_slove。 */
fp32 accel_offset[3];
/** 加速度计标定零漂存档。写：外部标定写入；读：外部标定加载（本文件内未直接使用）。 */
fp32 accel_cali_offset[3];

/** 磁力计(IST8310) 原始数据结构。写：ist8310 读取接口（本工程实际未启用磁力计 DMA）；读：imu_cali_slove（取 mag 原始值）。 */
ist8310_real_data_t ist8310_real_data;
/** 磁力计安装旋转矩阵。写：静态初值（IST8310_BOARD_INSTALL_SPIN_MATRIX）；读：imu_cali_slove。 */
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
/** 磁力计零漂（单位 uT）。写：外部标定写入；读：imu_cali_slove。 */
fp32 mag_offset[3];
/** 磁力计标定零漂存档。写：外部标定；读：外部标定（本文件内未直接使用）。 */
fp32 mag_cali_offset[3];

/** 温度闭环控制使能标志。意义：0=满功率加热未达标，1=达到 45℃ 转入 PID 闭环。写：imu_temp_control；读：imu_temp_control。 */
static uint8_t first_temperate;
/** 温度 PID 三参数 {KP,KI,KD}（来自 ins_task.h 宏）。写：静态初值；读：Imu_Task（pid_init 传入）。 */
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
/** 温度 PID 实例（目标 45℃）。写：pid_init（Imu_Task）、pid_calc（imu_temp_control）、imu_temp_control（钳 out/iout）；读：imu_temp_control。 */
pid_type_def imu_temp_pid;

/** AHRS 姿态解算采样周期（单位 s）=0.001（1ms）。写：静态初值；读：Imu_Task（AHRS_update 传入 time）。 */
static const float timing_time = 0.001f;

void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3]);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);
void get_angle(fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

/** 加速度计三阶低通滤波器一级状态(每个轴一个)。写：Imu_Task（滤波迭代）；读：Imu_Task。 */
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
/** 加速度计三阶低通滤波器二级状态。写：Imu_Task；读：Imu_Task。 */
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
/** 加速度计三阶低通滤波器三级状态（滤波输出）。写：Imu_Task；读：Imu_Task（作为 accel_fliter_3 传 AHRS_update）。 */
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
/** 三阶低通滤波系数 {a0,a1,a2}（IIR 二阶+一阶合成）。写：静态初值；读：Imu_Task（滤波迭代）。 */
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

/** 解算后机体坐标系陀螺仪角速度（单位 rad/s）。写：Imu_Task（imu_cali_slove 填充）；读：get_gyro_data_point、INS_cali_gyro（零漂估计）。 */
fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
/** 解算后机体坐标系加速度（单位 m/s2）。写：Imu_Task（imu_cali_slove 填充）；读：get_accel_data_point（滤波后的 accel_fliter_3 用于 AHRS）。 */
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
/** 解算后机体坐标系磁力计（单位 uT）。写：Imu_Task（imu_cali_slove 填充）；读：get_mag_data_point（本工程 AHRS 未使用）。 */
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
/** 姿态四元数 [w,x,y,z]。写：AHRS_init（置单位四元数）、AHRS_update（Mahony 更新）；读：get_INS_quat_point、get_angle（解欧拉角）。 */
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
/** 姿态欧拉角 [yaw,pitch,roll]（单位 rad）。写：Imu_Task（get_angle 填充）；读：get_INS_angle_point。 */
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};
/** 偏航角 yaw（rad），全局导出版本。写：Imu_Task；读：外部模块（VOFA 上传/UI/底盘控制）。 */
fp32 yaw = 0.0f;
/** 俯仰角 pitch（rad），全局导出版本。写：Imu_Task（取 INS_angle[ROLL 偏移]，与 roll 同源，属原逻辑照搬）；读：外部模块。 */
fp32 pitch = 0.0f;
/** 横滚角 roll（rad），全局导出版本。写：Imu_Task；读：外部模块。 */
fp32 roll = 0.0f;

/**
 * @brief   IMU 数据采集任务（CMSIS-RTOS2 线程）
 * @param   argument  线程入口参数；osThreadNew 创建时传入 NULL，函数未使用
 * @return  无（主循环为死循环，不返回）
 * @note    初始化 BMI088/IST8310、SPI1 DMA、温控 PID、AHRS，并把本线程 ID 存入
 *          INS_task_local_handler；进入主循环用 osThreadFlagsWait 等待标志（由
 *          HAL_GPIO_EXTI_Callback 的 GPIO_PIN_0 软中断 osThreadFlagsSet 唤醒），
 *          依次解析 gyro/accel/temp，经 imu_cali_slove 旋转+零漂、三阶低通滤波、
 *          AHRS 融合解算欧拉角，刷新 INS_gyro/INS_accel/INS_mag/INS_quat/INS_angle
 *          与全局 yaw/pitch/roll。
 */
void Imu_Task(void * argument)
{
    osDelay(INS_TASK_INIT_TIME);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    while(BMI088_init())
    {
        osDelay(100);
    }
    while(ist8310_init())
    {
        osDelay(100);
    }
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
    pid_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    AHRS_init(INS_quat, INS_accel, INS_mag);
    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
    INS_task_local_handler = osThreadGetId();
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    imu_start_dma_flag = 1;
    while (1)
    {
        // 等待 SPI DMA 完成(由 HAL_SPI_TxRxCpltCallback 软触发 GPIO_PIN_0)
        osThreadFlagsWait(INS_TASK_NOTIFY_FLAG, osFlagsWaitAny, osWaitForever);
			
        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }

        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];
        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];
        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];
        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

        AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
        yaw = *(INS_angle + INS_YAW_ADDRESS_OFFSET);
        pitch = *(INS_angle + INS_ROLL_ADDRESS_OFFSET);
        roll = *(INS_angle + INS_ROLL_ADDRESS_OFFSET);
        // 本工程实际未使用磁力计
        if(mag_update_flag &= 1 << IMU_DR_SHFITS)
        {
            mag_update_flag &= ~(1 << IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);
        }
    }
}

/**
 * @brief   初始化姿态四元数为初值（单位四元数 w=1, x=y=z=0）
 * @param   quat  四元数数组 [w,x,y,z]；要求传入长度≥4 的 fp32 数组，函数内被写为初始姿态
 * @param   accel 加速度计数据（本函数不使用，仅占位）
 * @param   mag   磁力计数据（本函数不使用，仅占位）
 * @return  无
 */
void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

/**
 * @brief   Mahony 互补滤波姿态更新（封装 MahonyAHRSupdate）
 * @param   quat  四元数 [w,x,y,z]；要求传入当前姿态，函数内被更新为最新姿态
 * @param   time  姿态采样周期（单位 s），要求传入实际周期（本工程恒为 timing_time=0.001）
 * @param   gyro  机体角速度（单位 rad/s），要求传入长度≥3 数组
 * @param   accel 机体加速度（单位 m/s2，建议先低通滤波），要求传入长度≥3 数组
 * @param   mag   磁力计（单位 uT；本工程未启用磁力计，要求传入但不参与）
 * @return  无
 */
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}

/**
 * @brief   由四元数解算欧拉角 yaw/pitch/roll（atan2f/asinf 标准公式）
 * @param   q      姿态四元数 [w,x,y,z]，要求传入长度≥4 数组（通常传 INS_quat）
 * @param   yaw    偏航角输出指针；要求传入有效 fp32 变量地址，函数内写入结果（单位 rad）
 * @param   pitch  俯仰角输出指针；要求传入有效 fp32 变量地址，函数内写入结果（单位 rad）
 * @param   roll   横滚角输出指针；要求传入有效 fp32 变量地址，函数内写入结果（单位 rad）
 * @return  无（结果通过三个指针回传）
 */
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

/**
 * @brief   设置温度加热 PWM 占空比（写 TIM10 CH1 比较值）
 * @param   pwm  PWM 比较值，要求 0≤pwm≤MPU6500_TEMP_PWM_MAX；越大加热越强
 * @return  无
 */
static void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}

/**
 * @brief   将 BMI088/IST8310 原始值经安装旋转矩阵 + 零漂换算到机体坐标系
 * @param   gyro    陀螺仪输出数组（rad/s）；要求传入长度≥3 数组，函数内写入结果
 * @param   accel   加速度计输出数组（m/s2）；要求传入长度≥3 数组，函数内写入结果
 * @param   mag     磁力计输出数组（uT）；要求传入长度≥3 数组，函数内写入结果
 * @param   bmi088  BMI088 原始数据结构指针；要求传入已填充 gyro/accel 的有效指针
 * @param   ist8310 IST8310 原始数据结构指针；要求传入已填充 mag 的有效指针
 * @return  无
 */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}

/**
 * @brief   BMI088 内置温度闭环控制（目标 45℃）
 * @param   temp  当前温度（单位 ℃），要求传入传感器实时温度
 * @return  无
 * @note    未达标(标志 first_temperate==0)时满功率加热并累计稳定时间；累计超 200 次
 *          且温度>45℃ 则置 first_temperate=1 转入 PID 闭环，输出钳位到 ≥0。
 */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        pid_calc(&imu_temp_pid, temp, 45);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        imu_pwm_set(tempPWM);
    }
    else
    {
        // 未达到设定温度，一直满功率加热
        if (temp > 45)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                // 达到目标温度，开启闭环控制(累计加热时间)
                first_temperate = 1;
                imu_temp_pid.iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }
        imu_pwm_set(MPU6500_TEMP_PWM_MAX - 1);
    }
}

/**
 * @brief   陀螺仪零漂迭代估计：每次 gyro_offset -= 0.0003f × gyro（静止时收敛到零漂）
 * @param   gyro_offset       零漂数组（rad/s）；要求传入长度≥3 数组，函数内被更新
 * @param   gyro              当前角速度（rad/s），要求传入长度≥3 数组
 * @param   offset_time_count 计数指针；要求传入有效 uint16 变量地址，函数内自增
 * @return  无
 */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }
    gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
    gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
    gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
    (*offset_time_count)++;
}

/**
 * @brief   陀螺仪零漂标定入口：首次数从存档加载零漂，之后每拍迭代更新
 * @param   cali_scale  标定尺度输出数组；要求传入长度≥3 数组，函数内写为 1.0f
 * @param   cali_offset 标定零漂输出数组（rad/s）；要求传入长度≥3 数组，函数内写入当前累计零漂
 * @param   time_count  迭代计数指针；要求传入有效 uint16 变量地址；为 0 时先加载 gyro_cali_offset 存档
 * @return  无
 */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
    if( *time_count == 0)
    {
        gyro_offset[0] = gyro_cali_offset[0];
        gyro_offset[1] = gyro_cali_offset[1];
        gyro_offset[2] = gyro_cali_offset[2];
    }
    gyro_offset_calc(gyro_offset, INS_gyro, time_count);
    cali_offset[0] = gyro_offset[0];
    cali_offset[1] = gyro_offset[1];
    cali_offset[2] = gyro_offset[2];
    cali_scale[0] = 1.0f;
    cali_scale[1] = 1.0f;
    cali_scale[2] = 1.0f;
}

/**
 * @brief   保存标定零漂到存档并立即应用到当前零漂
 * @param   cali_scale  标定尺度输入数组；要求传入长度≥3 数组，本函数不使用（保留接口）
 * @param   cali_offset 标定零漂输入数组（rad/s）；要求传入长度≥3 数组，函数内读取
 * @return  无
 */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}

/**
 * @brief   获取姿态四元数数组指针（供外部模块只读）
 * @return  指向 INS_quat[4] 的 const 指针；无入参
 */
const fp32 *get_INS_quat_point(void)
{
    return INS_quat;
}

/**
 * @brief   获取姿态欧拉角数组指针（供外部模块只读）
 * @return  指向 INS_angle[3]（[yaw,pitch,roll]）的 const 指针；无入参
 */
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}

/**
 * @brief   获取解算后机体角速度数组指针（供外部模块只读）
 * @return  指向 INS_gyro[3] 的 const 指针；无入参
 */
extern const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}

/**
 * @brief   获取解算后机体加速度数组指针（供外部模块只读）
 * @return  指向 INS_accel[3] 的 const 指针；无入参
 */
extern const fp32 *get_accel_data_point(void)
{
    return INS_accel;
}

/**
 * @brief   获取解算后磁力计数组指针（供外部模块只读；本工程磁力计未启用）
 * @return  指向 INS_mag[3] 的 const 指针；无入参
 */
extern const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}

/**
 * @brief   GPIO 外部中断回调（HAL），处理 BMI088/IST8310 数据就绪与软中断唤醒
 * @param   GPIO_Pin  触发引脚：INT1_ACCEL_Pin（加计就绪）、INT1_GYRO_Pin（陀螺就绪）、
 *                    IST8310_DRDY_Pin（磁力计就绪，本工程不用）、GPIO_PIN_0（Imu_Task 软唤醒）
 * @return  无
 * @note    加计/陀螺就绪时置对应 DR 标志；若 imu_start_dma_flag 已置位则调 imu_cmd_spi_dma
 *          发起 DMA 采样；GPIO_PIN_0 分支在内核运行后用 osThreadFlagsSet 唤醒 Imu_Task。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == IST8310_DRDY_Pin)
    {
        mag_update_flag |= 1 << IMU_DR_SHFITS;
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {
        // 唤醒 Imu_Task
        if (osKernelGetState() == osKernelRunning)
        {
            osThreadFlagsSet(INS_task_local_handler, INS_TASK_NOTIFY_FLAG);
        }
    }
}

/**
 * @brief   按 DR 就绪标志发起下一次 SPI1 DMA 采样（陀螺/加计/温度三路之一）
 * @return  无
 * @note    仅当对应 DR 置位、TX/RX DMA 均空闲(EN=0)、且无其它路 SPI 传输在忙时发起；
 *          发起前拉低对应片选并调 SPI1_DMA_enable。陀螺→加计→温度按优先级仲裁。
 */
static void imu_cmd_spi_dma(void)
{
    // 发起陀螺仪 DMA 读取
    if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);
        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }
    // 发起加速度计 DMA 读取
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }
    // 发起温度 DMA 读取
    if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        return;
    }
}

/**
 * @brief   SPI1 DMA 收发完成回调(HAL_SPI_TransmitReceive_DMA 触发)
 * @param   hspi  SPI 句柄；要求传入当前完成传输的 SPI 句柄（本函数仅对 SPI1 处理）
 * @return  无
 * @note    一次收发(RX DMA TC)完成后由 HAL 调用; 按 gyro/accel/temp 更新标志位,
 *          拉高对应片选, 再发起下一次采样(imu_cmd_spi_dma), 若陀螺仪已更新则软触发
 *          GPIO_PIN_0 唤醒 Imu_Task. 原 DMA2_Stream2_IRQHandler 逻辑迁移至此.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI1)
    {
        //gyro read over
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }
        //accel read over
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        imu_cmd_spi_dma();
        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

/**
 * @brief   初始化 SPI1 的 TX/RX DMA 请求并使能 SPI1
 * @param   tx_buf  发送缓冲地址（本实现未使用，仅保留接口）
 * @param   rx_buf  接收缓冲地址（本实现未使用，仅保留接口）
 * @param   num     传输长度（本实现未使用，仅保留接口）
 * @return  无
 * @note    SPI1_DMA_init 需在 Imu_Task 完成 HAL_SPI_Init 后调用；SPI 的 RX/TX DMA 请求
 *          由 CubeMX 在 MX_SPI1_Init 中已配置（hdmarx/hdmatx）。
 */
void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num)
{
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);
    __HAL_SPI_ENABLE(&hspi1);
}

/**
 * @brief   发起一次 SPI1 全双工 DMA 收发
 * @param   tx_buf  发送缓冲地址；要求传入已填充指令帧的缓冲（如 gyro_dma_tx_buf）
 * @param   rx_buf  接收缓冲地址；要求传入足够容纳 ndtr 字节的缓冲（如 gyro_dma_rx_buf）
 * @param   ndtr    本次传输数据个数；要求传入对应通道长度（如 SPI_DMA_GYRO_LENGHT）
 * @return  无（HAL 返回状态未检查；若发起失败无 DMA 完成标志，需核对缓冲地址与长度）
 */
void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr)
{
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)tx_buf, (uint8_t*)rx_buf, ndtr);
}
