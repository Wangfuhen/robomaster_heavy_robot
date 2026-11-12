/*
  RM串口协议
	更新于2025.3.2
	添加了文字注释。需及时更新，当前版本V1.7.0
	
*/
#ifndef REFEREE_H
#define REFEREE_H
#include "main.h"
#include "protocol.h"

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;

typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;

typedef __packed struct        //0x001
{
	uint8_t game_type : 4;     //比赛类型
	uint8_t game_progress : 4; //当前比赛阶段
	uint16_t stage_remain_time;//当前阶段剩余时间 秒
	uint64_t SyncTimeStamp;    //UNIX时间
} ext_game_state_t;

typedef __packed struct    //0x002
{
    uint8_t winner;        //比赛结果 0平1红2蓝胜
} ext_game_result_t;

typedef __packed struct
{
	uint16_t red_1_robot_HP;//对应机器人的血量
	uint16_t red_2_robot_HP; 
	uint16_t red_3_robot_HP; 
	uint16_t red_4_robot_HP; 
	uint16_t reserved;       //5号步兵没有了 保留
	uint16_t red_7_robot_HP; 
	uint16_t red_outpost_HP; //前哨战血量
	uint16_t red_base_HP;    //基地血量
	uint16_t blue_1_robot_HP; 
	uint16_t blue_2_robot_HP; 
	uint16_t blue_3_robot_HP; 
	uint16_t blue_4_robot_HP; 
	uint16_t reserved1;      //5号步兵没有了 保留
	uint16_t blue_7_robot_HP; 
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;

typedef __packed struct      //0x101
{
	uint32_t event_type;     //比赛中的事件，增益 飞镖命中情况等 需要自己去查各个字节内容
} ext_event_data_t;

typedef __packed struct      //0x0102  今年没了
{
    uint8_t reserved;
    uint8_t supply_robot_id;//补弹机器人ID
    uint8_t supply_projectile_step;//出弹口开闭状态
    uint8_t supply_projectile_num;//补弹数量
} ext_supply_projectile_action_t;


typedef __packed struct     //0x0103 今年没有了
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct    //0x0104
{
    uint8_t level;        //己方最后一次受到判罚的等级 1双方黄牌 2黄牌 3红牌 4判负
    uint8_t foul_robot_id;//己方最后一次受到判罚的违规机器人ID  （判负和双方黄牌时，该值为0）
	uint8_t count;        //己方最后一次受到判罚的违规机器人对应判罚等级的违规次数
} ext_referee_warning_t;

typedef __packed struct           //0x0105
{
	uint8_t dart_remaining_time;  //己方飞镖发射剩余时间
	uint16_t dart_info;           //飞镖击中信息
} ext_dart_remaining_time_t;

typedef __packed struct //0x0201
{
	uint8_t robot_id;   //本机器人ID 
	uint8_t robot_level;//机器人等级 
	uint16_t current_HP;//机器人当前血量
	uint16_t maximum_HP;//机器人血量上限
	uint16_t shooter_barrel_cooling_value;       //机器人枪口热量每秒冷却值 
	uint16_t shooter_barrel_heat_limit;          //机器人枪口热量上限 
	uint16_t chassis_power_limit;                //机器人底盘功率上限
	uint8_t power_management_gimbal_output : 1;  //电源管理模块云台 底盘 发射机构 24v输出情况（需要具体按位分析解包）
	uint8_t power_management_chassis_output : 1;  
	uint8_t power_management_shooter_output : 1; 
} ext_game_robot_state_t;

typedef __packed struct //0x0202
{
	uint16_t reserved;                       //没了
	uint16_t reserved1;                      //没了
	float reserved2;                         //底盘功率保留没了
	uint16_t buffer_energy;                  //缓冲能量
	uint16_t shooter_17mm_1_barrel_heat;     //第1个17mm发射机构的枪口热量
	uint16_t shooter_17mm_2_barrel_heat;     //第2个17mm发射机构的枪口热量
	uint16_t shooter_42mm_barrel_heat;       //42mm发射机构的枪口热量
} ext_power_heat_data_t;

typedef __packed struct //0x0203
{
    float x;      //本机器人位置x坐标，单位：m 
    float y;      //本机器人位置y坐标，单位：m 
	float angle;  //本机器人测速模块的朝向，单位：度。正北为0度 
} ext_game_robot_pos_t;

typedef __packed struct //0x0204
{
    uint8_t recovery_buff;      //机器人回血增益（百分比，值为10表示每秒恢复血量上限的10%）
	uint8_t cooling_buff;       //机器人枪口冷却倍率（直接值，值为5表示5倍冷却）
	uint8_t defence_buff;       //机器人防御增益（百分比，值为50表示50%防御增益）
	uint8_t vulnerability_buff; //机器人负防御增益（百分比，值为30表示-30%防御增益）
	uint16_t attack_buff;       //机器人攻击增益（百分比，值为50表示50%攻击增益）
} ext_buff_musk_t;

typedef __packed struct //0x0205  这个没了
{
	uint8_t airforce_status; //空中机器人状态（0为正在冷却，1为冷却完毕，2为正在空中支援）
	uint8_t time_remain;     //此状态的剩余时间（单位为：秒，向下取整，即冷却时间剩余1.9秒时，此值为1）
} aerial_robot_energy_t;

typedef __packed struct //0x0206
{
    uint8_t armor_id : 4;  //扣血原因（需要按位分析解包）
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct //0x0207
{
    uint8_t bullet_type; //弹丸类型：1：17mm弹丸 2：42mm弹丸
    uint8_t shooter_id;  //发射机构ID 1：第1个17mm发射机构 2：第2个17mm发射机构 3：42mm发射机构
    uint8_t bullet_freq; //弹丸射速（单位：Hz）
    float bullet_speed;  //弹丸初速度（单位：m/s）
} ext_shoot_data_t;

typedef __packed struct//0x208
{
    uint16_t bullet_remaining_num_17mm; //17mm弹丸允许发弹量 
    uint16_t bullet_remaining_num_42mm; //42mm弹丸允许发弹量
	uint16_t coin_remaining_num;        //剩余金币数量 
} ext_bullet_remaining_t;

typedef __packed struct   //0x209
{
	uint32_t rfid_status; //检测的增益点RFID卡（需要按位分析解包）
} ext_rfid_status_t;

typedef __packed struct   //0x20A
{ 
	uint8_t dart_launch_opening_status; //当前飞镖发射站的状态  1：关闭 2：正在开启或者关闭中 0：已经开启
	uint8_t reserved;                   //保留
	uint16_t target_change_time;        //切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为0
	uint16_t latest_launch_cmd_time;    //最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为0
}dart_client_cmd_t; 
typedef __packed struct //0x20B
{ 
	float hero_x;       //己方对应机器人位置x轴坐标，单位：m
	float hero_y;       //己方对应机器人位置y轴坐标，单位：m
	float engineer_x;   
	float engineer_y;  
	float standard_3_x;  
	float standard_3_y;  
	float standard_4_x;  
	float standard_4_y;  
	float reserved;     //5号步兵没了
	float reserved1;    //5号步兵没了
}ground_robot_position_t;
typedef __packed struct //0x20C
{ 
   uint8_t mark_progress;  //对方机器人易伤情况（需要按位分析解包） 英雄 工程 步兵 步兵 哨兵 
}radar_mark_data_t; 

typedef __packed struct //0x20D
{ 
	uint32_t sentry_info;  //哨兵兑换发弹 买活所需金币 （需要按位分析解包）
    int16_t sentry_info_2; //哨兵是否脱战 17mm允许发弹 （需要按位分析解包）
} sentry_info_t; 
typedef __packed struct //0x20E
{ 
	uint8_t radar_info; //雷达易伤信息（需要按位解包）
} radar_info_t; 
typedef __packed struct //0x0301
{
	uint16_t data_cmd_id;//子内容ID
    uint16_t send_ID;    //发送者ID 
    uint16_t receiver_ID;//接收者ID
    uint8_t data[112];   //内容数据段
} ext_student_interactive_data_t;

typedef struct
{
	ext_game_state_t game_state;//比赛状态数据，1Hz 周期发送
	ext_game_result_t game_result;//比赛结果数据，比赛结束后发送
	ext_game_robot_HP_t game_robot_HP_t;//比赛机器人血量数据，1Hz 周期发送
	ext_event_data_t field_event;//场地事件数据，1Hz 周期发送
	ext_supply_projectile_action_t supply_projectile_action_t;//场地补给站动作标识数据，动作改变后发送
	ext_supply_projectile_booking_t supply_projectile_booking_t;//请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）
	ext_referee_warning_t referee_warning_t;//裁判警告数据，警告发生后发送
	ext_game_robot_state_t robot_state;//机器人状态数据，10Hz 周期发送
	ext_power_heat_data_t power_heat_data_t;//实时功率热量数据，50Hz 周期发送
	ext_game_robot_pos_t game_robot_pos_t;//机器人位置数据，10Hz 发送
	ext_buff_musk_t buff_musk_t;//机器人增益数据，增益状态改变后发送
	aerial_robot_energy_t robot_energy_t;//空中机器人的状态
	ext_robot_hurt_t robot_hurt_t;//机器人掉血原因
	ext_shoot_data_t shoot_data_t;//发射信息
	ext_bullet_remaining_t bullet_remaining_t;//发弹量和金币多少
	ext_rfid_status_t rfid_status;//RFID卡检测状态
	dart_client_cmd_t dart_client_cmd;//飞镖信息
	ground_robot_position_t ground_robot_position;//我方机器人的位置
	radar_mark_data_t radar_mark_data;//雷达标记敌方机器人进度
	sentry_info_t sentry_info;//哨兵信息
	radar_info_t radar_info;//雷达信息
	ext_student_interactive_data_t student_interactive_data_t;//机器人之间通信
}RM_Referee_system_t;


extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(fp32 *buffer,fp32 *limit);

extern RM_Referee_system_t RM_Referee_system;

extern uint8_t get_robot_id(void);
extern const RM_Referee_system_t *get_Referee_point(void);
extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0);
extern void get_shoot_heat0_limit_and_heat_cooling_rate(uint16_t *heat0_limit, uint16_t *rate);
#endif
