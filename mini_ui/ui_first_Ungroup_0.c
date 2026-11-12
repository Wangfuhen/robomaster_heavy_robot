//
// Created by RM UI Designer
//

#include "ui_first_Ungroup_0.h"
#include "string.h"

#define FRAME_ID 1
#define GROUP_ID 0
#define START_ID 0

ui_string_frame_t ui_first_Ungroup_0;

ui_interface_string_t* ui_first_Ungroup_Ares = &ui_first_Ungroup_0.option;

void _ui_init_first_Ungroup_0() {
    ui_first_Ungroup_0.option.figure_name[0] = FRAME_ID;
    ui_first_Ungroup_0.option.figure_name[1] = GROUP_ID;
    ui_first_Ungroup_0.option.figure_name[2] = START_ID;
    ui_first_Ungroup_0.option.operate_tpyel = 1;
    ui_first_Ungroup_0.option.figure_tpye = 7;
    ui_first_Ungroup_0.option.layer = 0;
    ui_first_Ungroup_0.option.font_size = 35;
    ui_first_Ungroup_0.option.start_x = 875;
    ui_first_Ungroup_0.option.start_y = 677;
    ui_first_Ungroup_0.option.color = 2;
    ui_first_Ungroup_0.option.str_length = 4;
    ui_first_Ungroup_0.option.width = 4;
    strcpy(ui_first_Ungroup_Ares->string, "Ares");

    ui_proc_string_frame(&ui_first_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_first_Ungroup_0, sizeof(ui_first_Ungroup_0));
}

void _ui_update_first_Ungroup_0() {
    ui_first_Ungroup_0.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_first_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_first_Ungroup_0, sizeof(ui_first_Ungroup_0));
}

void _ui_remove_first_Ungroup_0() {
    ui_first_Ungroup_0.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_first_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_first_Ungroup_0, sizeof(ui_first_Ungroup_0));
}