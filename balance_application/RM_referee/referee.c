#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;
RM_Referee_system_t RM_Referee_system;
const RM_Referee_system_t *get_Referee_point(void);
const RM_Referee_system_t *get_Referee_point(void)
{
    return &RM_Referee_system;
}



void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));
    memset(&RM_Referee_system.game_state, 0, sizeof(ext_game_state_t));
    memset(&RM_Referee_system.game_result, 0, sizeof(ext_game_result_t));
    memset(&RM_Referee_system.game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));


    memset(&RM_Referee_system.field_event, 0, sizeof(ext_event_data_t));
    memset(&RM_Referee_system.supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&RM_Referee_system.supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&RM_Referee_system.referee_warning_t, 0, sizeof(ext_referee_warning_t));


    memset(&RM_Referee_system.robot_state, 0, sizeof(ext_game_robot_state_t));
    memset(&RM_Referee_system.power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&RM_Referee_system.game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&RM_Referee_system.buff_musk_t, 0, sizeof(ext_buff_musk_t));
    memset(&RM_Referee_system.robot_energy_t, 0, sizeof(aerial_robot_energy_t));
    memset(&RM_Referee_system.robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&RM_Referee_system.shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&RM_Referee_system.bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));


    memset(&RM_Referee_system.student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));

}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID://0x0001
        {
            memcpy(&RM_Referee_system.game_state, frame + index, sizeof(ext_game_state_t));
        }
        break;
        case GAME_RESULT_CMD_ID://0x0002
        {
            memcpy(&RM_Referee_system.game_result, frame + index, sizeof(RM_Referee_system.game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID://0x0003
        {
            memcpy(&RM_Referee_system.game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;


        case FIELD_EVENTS_CMD_ID://0x0101
        {
            memcpy(&RM_Referee_system.field_event, frame + index, sizeof(RM_Referee_system.field_event));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID://0x0102
        {
            memcpy(&RM_Referee_system.supply_projectile_action_t, frame + index, sizeof(RM_Referee_system.supply_projectile_action_t));
        }
        break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID://0x0103
        {
            memcpy(&RM_Referee_system.supply_projectile_booking_t, frame + index, sizeof(RM_Referee_system.supply_projectile_booking_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID://0x0104
        {
            memcpy(&RM_Referee_system.referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
        }
        break;

        case ROBOT_STATE_CMD_ID://0x0201
        {
            memcpy(&RM_Referee_system.robot_state, frame + index, sizeof(RM_Referee_system.robot_state));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID://0x202
        {
            memcpy(&RM_Referee_system.power_heat_data_t, frame + index, sizeof(RM_Referee_system.power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID://0x203
        {
            memcpy(&RM_Referee_system.game_robot_pos_t, frame + index, sizeof(RM_Referee_system.game_robot_pos_t));
        }
        break;
        case BUFF_MUSK_CMD_ID://0x204
        {
            memcpy(&RM_Referee_system.buff_musk_t, frame + index, sizeof(RM_Referee_system.buff_musk_t));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID://0x205
        {
            memcpy(&RM_Referee_system.robot_energy_t, frame + index, sizeof(RM_Referee_system.robot_energy_t));
        }
        break;
        case ROBOT_HURT_CMD_ID://0x206
        {
            memcpy(&RM_Referee_system.robot_hurt_t, frame + index, sizeof(RM_Referee_system.robot_hurt_t));
        }
        break;
        case SHOOT_DATA_CMD_ID://0x207
        {
            memcpy(&RM_Referee_system.shoot_data_t, frame + index, sizeof(RM_Referee_system.shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID://0x208
        {
            memcpy(&RM_Referee_system.bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
        }
        break;
		case ROBOT_RFID_STATUS_CMD_ID://0x209
        {
            memcpy(&RM_Referee_system.rfid_status, frame + index, sizeof(ext_rfid_status_t));
        }
        break;
		case INSTRUCTION_BOOK_FOR_DART_CMD_ID://0x20A
        {
            memcpy(&RM_Referee_system.dart_client_cmd, frame + index, sizeof(dart_client_cmd_t));
        }
        break;
		case ALL_ROBOT_POS_CMD_ID://0x20B
        {
            memcpy(&RM_Referee_system.ground_robot_position, frame + index, sizeof(ext_rfid_status_t));
        }
        break;
		case ALL_ROBOT_RADAR_SIGNATURE_CMD_ID://0x20C
        {
            memcpy(&RM_Referee_system.radar_mark_data, frame + index, sizeof(radar_mark_data_t));
        }
        break;
		case SENTRY_ROBOT_EXCHANGE_CMD_ID://0x20D
        {
            memcpy(&RM_Referee_system.sentry_info, frame + index, sizeof(sentry_info_t));
        }
        break;
		case RADAR_DOUBLE_HURT_CMD_ID://0x20E
        {
            memcpy(&RM_Referee_system.radar_info, frame + index, sizeof(radar_info_t));
        }
        break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID://0x301
        {
            memcpy(&RM_Referee_system.student_interactive_data_t, frame + index, sizeof(RM_Referee_system.student_interactive_data_t));
        }
        break;
        default:
        {
            break;
        }
    }
}

void get_chassis_power_and_buffer(fp32 *buffer,fp32 *limit)
{
    *buffer = RM_Referee_system.power_heat_data_t.buffer_energy;
    *limit = RM_Referee_system.robot_state.chassis_power_limit;
}


uint8_t get_robot_id(void)
{
    return RM_Referee_system.robot_state.robot_id;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    *heat0_limit = RM_Referee_system.robot_state.shooter_barrel_heat_limit;
    *heat0 = RM_Referee_system.power_heat_data_t.shooter_42mm_barrel_heat;
}

void get_shoot_heat0_limit_and_heat_cooling_rate(uint16_t *heat0_limit, uint16_t *rate)
{
    *heat0_limit = RM_Referee_system.robot_state.shooter_barrel_heat_limit;
    *rate =RM_Referee_system.robot_state.shooter_barrel_cooling_value;
}

