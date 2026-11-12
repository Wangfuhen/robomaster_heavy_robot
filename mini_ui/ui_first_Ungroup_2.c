//
// Created by RM UI Designer
//

#include "ui_first_Ungroup_2.h"

#define FRAME_ID 1
#define GROUP_ID 0
#define START_ID 8
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_first_Ungroup_2;
ui_interface_line_t *ui_first_Ungroup_lne_6 = (ui_interface_line_t *)&(ui_first_Ungroup_2.data[0]);
ui_interface_line_t *ui_first_Ungroup_lne_7 = (ui_interface_line_t *)&(ui_first_Ungroup_2.data[1]);
ui_interface_line_t *ui_first_Ungroup_ok_5 = (ui_interface_line_t *)&(ui_first_Ungroup_2.data[2]);
ui_interface_line_t *ui_first_Ungroup_ok_6 = (ui_interface_line_t *)&(ui_first_Ungroup_2.data[3]);
ui_interface_ellipse_t *ui_first_Ungroup_NewEllipse = (ui_interface_ellipse_t *)&(ui_first_Ungroup_2.data[4]);
ui_interface_line_t *ui_first_Ungroup_lne_8 = (ui_interface_line_t *)&(ui_first_Ungroup_2.data[5]);
ui_interface_rect_t *ui_first_Ungroup_NewRect = (ui_interface_rect_t *)&(ui_first_Ungroup_2.data[6]);

void _ui_init_first_Ungroup_2() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_first_Ungroup_2.data[i].figure_name[0] = FRAME_ID;
        ui_first_Ungroup_2.data[i].figure_name[1] = GROUP_ID;
        ui_first_Ungroup_2.data[i].figure_name[2] = i + START_ID;
        ui_first_Ungroup_2.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_first_Ungroup_2.data[i].operate_tpyel = 0;
    }

    ui_first_Ungroup_lne_6->figure_tpye = 0;
    ui_first_Ungroup_lne_6->layer = 2;
    ui_first_Ungroup_lne_6->start_x = 941;
    ui_first_Ungroup_lne_6->start_y = 444;
    ui_first_Ungroup_lne_6->end_x = 941;
    ui_first_Ungroup_lne_6->end_y = 526;
    ui_first_Ungroup_lne_6->color = 1;
    ui_first_Ungroup_lne_6->width = 3;

    ui_first_Ungroup_lne_7->figure_tpye = 0;
    ui_first_Ungroup_lne_7->layer = 2;
    ui_first_Ungroup_lne_7->start_x = 688;
    ui_first_Ungroup_lne_7->start_y = 123;
    ui_first_Ungroup_lne_7->end_x = 1193;
    ui_first_Ungroup_lne_7->end_y = 123;
    ui_first_Ungroup_lne_7->color = 6;
    ui_first_Ungroup_lne_7->width = 7;

    ui_first_Ungroup_ok_5->figure_tpye = 0;
    ui_first_Ungroup_ok_5->layer = 0;
    ui_first_Ungroup_ok_5->start_x = 912;
    ui_first_Ungroup_ok_5->start_y = 479;
    ui_first_Ungroup_ok_5->end_x = 974;
    ui_first_Ungroup_ok_5->end_y = 479;
    ui_first_Ungroup_ok_5->color = 7;
    ui_first_Ungroup_ok_5->width = 2;

    ui_first_Ungroup_ok_6->figure_tpye = 0;
    ui_first_Ungroup_ok_6->layer = 0;
    ui_first_Ungroup_ok_6->start_x = 925;
    ui_first_Ungroup_ok_6->start_y = 523;
    ui_first_Ungroup_ok_6->end_x = 958;
    ui_first_Ungroup_ok_6->end_y = 523;
    ui_first_Ungroup_ok_6->color = 2;
    ui_first_Ungroup_ok_6->width = 2;

    ui_first_Ungroup_NewEllipse->figure_tpye = 3;
    ui_first_Ungroup_NewEllipse->layer = 0;
    ui_first_Ungroup_NewEllipse->rx = 74;
    ui_first_Ungroup_NewEllipse->ry = 62;
    ui_first_Ungroup_NewEllipse->start_x = 940;
    ui_first_Ungroup_NewEllipse->start_y = 486;
    ui_first_Ungroup_NewEllipse->color = 7;
    ui_first_Ungroup_NewEllipse->width = 5;

    ui_first_Ungroup_lne_8->figure_tpye = 0;
    ui_first_Ungroup_lne_8->layer = 1;
    ui_first_Ungroup_lne_8->start_x = 885;
    ui_first_Ungroup_lne_8->start_y = 456;
    ui_first_Ungroup_lne_8->end_x = 998;
    ui_first_Ungroup_lne_8->end_y = 456;
    ui_first_Ungroup_lne_8->color = 8;
    ui_first_Ungroup_lne_8->width = 3;

    ui_first_Ungroup_NewRect->figure_tpye = 1;
    ui_first_Ungroup_NewRect->layer = 0;
    ui_first_Ungroup_NewRect->start_x = 769;
    ui_first_Ungroup_NewRect->start_y = 390;
    ui_first_Ungroup_NewRect->color = 0;
    ui_first_Ungroup_NewRect->width = 3;
    ui_first_Ungroup_NewRect->end_x = 1106;
    ui_first_Ungroup_NewRect->end_y = 625;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_first_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_first_Ungroup_2, sizeof(ui_first_Ungroup_2));
}

void _ui_update_first_Ungroup_2() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_first_Ungroup_2.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_first_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_first_Ungroup_2, sizeof(ui_first_Ungroup_2));
}

void _ui_remove_first_Ungroup_2() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_first_Ungroup_2.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_first_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_first_Ungroup_2, sizeof(ui_first_Ungroup_2));
}
