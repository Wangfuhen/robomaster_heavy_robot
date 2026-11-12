//
// Created by RM UI Designer
//

#include "ui_first_Ungroup_1.h"

#define FRAME_ID 1
#define GROUP_ID 0
#define START_ID 1
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_first_Ungroup_1;
ui_interface_line_t *ui_first_Ungroup_ok_1 = (ui_interface_line_t *)&(ui_first_Ungroup_1.data[0]);
ui_interface_line_t *ui_first_Ungroup_ok_2 = (ui_interface_line_t *)&(ui_first_Ungroup_1.data[1]);
ui_interface_line_t *ui_first_Ungroup_line_1 = (ui_interface_line_t *)&(ui_first_Ungroup_1.data[2]);
ui_interface_line_t *ui_first_Ungroup_lne_2 = (ui_interface_line_t *)&(ui_first_Ungroup_1.data[3]);
ui_interface_line_t *ui_first_Ungroup_lne_3 = (ui_interface_line_t *)&(ui_first_Ungroup_1.data[4]);
ui_interface_line_t *ui_first_Ungroup_lne_4 = (ui_interface_line_t *)&(ui_first_Ungroup_1.data[5]);
ui_interface_line_t *ui_first_Ungroup_lne_5 = (ui_interface_line_t *)&(ui_first_Ungroup_1.data[6]);

void _ui_init_first_Ungroup_1() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_first_Ungroup_1.data[i].figure_name[0] = FRAME_ID;
        ui_first_Ungroup_1.data[i].figure_name[1] = GROUP_ID;
        ui_first_Ungroup_1.data[i].figure_name[2] = i + START_ID;
        ui_first_Ungroup_1.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_first_Ungroup_1.data[i].operate_tpyel = 0;
    }

    ui_first_Ungroup_ok_1->figure_tpye = 0;
    ui_first_Ungroup_ok_1->layer = 0;
    ui_first_Ungroup_ok_1->start_x = 662;
    ui_first_Ungroup_ok_1->start_y = 280;
    ui_first_Ungroup_ok_1->end_x = 755;
    ui_first_Ungroup_ok_1->end_y = 482;
    ui_first_Ungroup_ok_1->color = 1;
    ui_first_Ungroup_ok_1->width = 7;

    ui_first_Ungroup_ok_2->figure_tpye = 0;
    ui_first_Ungroup_ok_2->layer = 0;
    ui_first_Ungroup_ok_2->start_x = 1199;
    ui_first_Ungroup_ok_2->start_y = 293;
    ui_first_Ungroup_ok_2->end_x = 1121;
    ui_first_Ungroup_ok_2->end_y = 478;
    ui_first_Ungroup_ok_2->color = 1;
    ui_first_Ungroup_ok_2->width = 7;

    ui_first_Ungroup_line_1->figure_tpye = 0;
    ui_first_Ungroup_line_1->layer = 2;
    ui_first_Ungroup_line_1->start_x = 938;
    ui_first_Ungroup_line_1->start_y = 12;
    ui_first_Ungroup_line_1->end_x = 938;
    ui_first_Ungroup_line_1->end_y = 384;
    ui_first_Ungroup_line_1->color = 5;
    ui_first_Ungroup_line_1->width = 9;

    ui_first_Ungroup_lne_2->figure_tpye = 0;
    ui_first_Ungroup_lne_2->layer = 2;
    ui_first_Ungroup_lne_2->start_x = 885;
    ui_first_Ungroup_lne_2->start_y = 353;
    ui_first_Ungroup_lne_2->end_x = 998;
    ui_first_Ungroup_lne_2->end_y = 353;
    ui_first_Ungroup_lne_2->color = 3;
    ui_first_Ungroup_lne_2->width = 5;

    ui_first_Ungroup_lne_3->figure_tpye = 0;
    ui_first_Ungroup_lne_3->layer = 2;
    ui_first_Ungroup_lne_3->start_x = 833;
    ui_first_Ungroup_lne_3->start_y = 274;
    ui_first_Ungroup_lne_3->end_x = 1051;
    ui_first_Ungroup_lne_3->end_y = 274;
    ui_first_Ungroup_lne_3->color = 3;
    ui_first_Ungroup_lne_3->width = 7;

    ui_first_Ungroup_lne_4->figure_tpye = 0;
    ui_first_Ungroup_lne_4->layer = 2;
    ui_first_Ungroup_lne_4->start_x = 777;
    ui_first_Ungroup_lne_4->start_y = 193;
    ui_first_Ungroup_lne_4->end_x = 1104;
    ui_first_Ungroup_lne_4->end_y = 193;
    ui_first_Ungroup_lne_4->color = 6;
    ui_first_Ungroup_lne_4->width = 7;

    ui_first_Ungroup_lne_5->figure_tpye = 0;
    ui_first_Ungroup_lne_5->layer = 2;
    ui_first_Ungroup_lne_5->start_x = 918;
    ui_first_Ungroup_lne_5->start_y = 495;
    ui_first_Ungroup_lne_5->end_x = 969;
    ui_first_Ungroup_lne_5->end_y = 495;
    ui_first_Ungroup_lne_5->color = 5;
    ui_first_Ungroup_lne_5->width = 3;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_first_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_first_Ungroup_1, sizeof(ui_first_Ungroup_1));
}

void _ui_update_first_Ungroup_1() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_first_Ungroup_1.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_first_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_first_Ungroup_1, sizeof(ui_first_Ungroup_1));
}

void _ui_remove_first_Ungroup_1() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_first_Ungroup_1.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_first_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_first_Ungroup_1, sizeof(ui_first_Ungroup_1));
}
