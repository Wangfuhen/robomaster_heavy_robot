//
// Created by RM UI Designer
//

#include "ui_first_Ungroup_3.h"

#define FRAME_ID 1
#define GROUP_ID 0
#define START_ID 15
#define OBJ_NUM 2
#define FRAME_OBJ_NUM 2

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_first_Ungroup_3;
ui_interface_line_t *ui_first_Ungroup_round_1 = (ui_interface_line_t *)&(ui_first_Ungroup_3.data[0]);
ui_interface_line_t *ui_first_Ungroup_round_2 = (ui_interface_line_t *)&(ui_first_Ungroup_3.data[1]);

void _ui_init_first_Ungroup_3() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_first_Ungroup_3.data[i].figure_name[0] = FRAME_ID;
        ui_first_Ungroup_3.data[i].figure_name[1] = GROUP_ID;
        ui_first_Ungroup_3.data[i].figure_name[2] = i + START_ID;
        ui_first_Ungroup_3.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_first_Ungroup_3.data[i].operate_tpyel = 0;
    }

    ui_first_Ungroup_round_1->figure_tpye = 0;
    ui_first_Ungroup_round_1->layer = 1;
    ui_first_Ungroup_round_1->start_x = 537;
    ui_first_Ungroup_round_1->start_y = 9;
    ui_first_Ungroup_round_1->end_x = 657;
    ui_first_Ungroup_round_1->end_y = 269;
    ui_first_Ungroup_round_1->color = 5;
    ui_first_Ungroup_round_1->width = 14;

    ui_first_Ungroup_round_2->figure_tpye = 0;
    ui_first_Ungroup_round_2->layer = 1;
    ui_first_Ungroup_round_2->start_x = 1318;
    ui_first_Ungroup_round_2->start_y = 5;
    ui_first_Ungroup_round_2->end_x = 1200;
    ui_first_Ungroup_round_2->end_y = 274;
    ui_first_Ungroup_round_2->color = 5;
    ui_first_Ungroup_round_2->width = 14;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_first_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_first_Ungroup_3, sizeof(ui_first_Ungroup_3));
}

void _ui_update_first_Ungroup_3() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_first_Ungroup_3.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_first_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_first_Ungroup_3, sizeof(ui_first_Ungroup_3));
}

void _ui_remove_first_Ungroup_3() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_first_Ungroup_3.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_first_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_first_Ungroup_3, sizeof(ui_first_Ungroup_3));
}
