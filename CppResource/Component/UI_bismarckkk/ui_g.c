//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_5_frame_t ui_g_Ungroup_0;

ui_interface_number_t *ui_g_Ungroup_EGY_NUM = (ui_interface_number_t*)&(ui_g_Ungroup_0.data[0]);
ui_interface_ellipse_t *ui_g_Ungroup_LAUNCH3508 = (ui_interface_ellipse_t*)&(ui_g_Ungroup_0.data[1]);
ui_interface_ellipse_t *ui_g_Ungroup_AIM = (ui_interface_ellipse_t*)&(ui_g_Ungroup_0.data[2]);
ui_interface_ellipse_t *ui_g_Ungroup_AUTOROTATE = (ui_interface_ellipse_t*)&(ui_g_Ungroup_0.data[3]);

void _ui_init_g_Ungroup_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_Ungroup_0.data[i].figure_name[0] = 0;
        ui_g_Ungroup_0.data[i].figure_name[1] = 0;
        ui_g_Ungroup_0.data[i].figure_name[2] = i + 0;
        ui_g_Ungroup_0.data[i].operate_type = 1;
    }
    for (int i = 4; i < 5; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 0;
    }

    ui_g_Ungroup_EGY_NUM->figure_type = 6;
    ui_g_Ungroup_EGY_NUM->operate_type = 1;
    ui_g_Ungroup_EGY_NUM->layer = 0;
    ui_g_Ungroup_EGY_NUM->color = 1;
    ui_g_Ungroup_EGY_NUM->start_x = 538;
    ui_g_Ungroup_EGY_NUM->start_y = 155;
    ui_g_Ungroup_EGY_NUM->width = 4;
    ui_g_Ungroup_EGY_NUM->font_size = 40;
    ui_g_Ungroup_EGY_NUM->number = 1;

    ui_g_Ungroup_LAUNCH3508->figure_type = 3;
    ui_g_Ungroup_LAUNCH3508->operate_type = 1;
    ui_g_Ungroup_LAUNCH3508->layer = 0;
    ui_g_Ungroup_LAUNCH3508->color = 1;
    ui_g_Ungroup_LAUNCH3508->start_x = 1484;
    ui_g_Ungroup_LAUNCH3508->start_y = 590;
    ui_g_Ungroup_LAUNCH3508->width = 20;
    ui_g_Ungroup_LAUNCH3508->rx = 10;
    ui_g_Ungroup_LAUNCH3508->ry = 10;

    ui_g_Ungroup_AIM->figure_type = 3;
    ui_g_Ungroup_AIM->operate_type = 1;
    ui_g_Ungroup_AIM->layer = 0;
    ui_g_Ungroup_AIM->color = 1;
    ui_g_Ungroup_AIM->start_x = 1485;
    ui_g_Ungroup_AIM->start_y = 536;
    ui_g_Ungroup_AIM->width = 20;
    ui_g_Ungroup_AIM->rx = 10;
    ui_g_Ungroup_AIM->ry = 10;

    ui_g_Ungroup_AUTOROTATE->figure_type = 3;
    ui_g_Ungroup_AUTOROTATE->operate_type = 1;
    ui_g_Ungroup_AUTOROTATE->layer = 0;
    ui_g_Ungroup_AUTOROTATE->color = 1;
    ui_g_Ungroup_AUTOROTATE->start_x = 1483;
    ui_g_Ungroup_AUTOROTATE->start_y = 646;
    ui_g_Ungroup_AUTOROTATE->width = 20;
    ui_g_Ungroup_AUTOROTATE->rx = 10;
    ui_g_Ungroup_AUTOROTATE->ry = 10;


    ui_proc_5_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_update_g_Ungroup_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_remove_g_Ungroup_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}


void ui_init_g_Ungroup() {
    _ui_init_g_Ungroup_0();
}

void ui_update_g_Ungroup() {
    _ui_update_g_Ungroup_0();
}

void ui_remove_g_Ungroup() {
    _ui_remove_g_Ungroup_0();
}

