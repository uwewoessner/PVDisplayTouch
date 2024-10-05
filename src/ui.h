// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#ifndef _SQUARELINE_PROJECT_UI_H
#define _SQUARELINE_PROJECT_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

#include "ui_helpers.h"
#include "components/ui_comp.h"
#include "components/ui_comp_hook.h"
#include "ui_events.h"

// SCREEN: ui_PV
void ui_PV_screen_init(void);
extern lv_obj_t * ui_PV;
extern lv_obj_t * ui_TabView1;
extern lv_obj_t * ui_TabPage1;
extern lv_obj_t * ui_PreisLabel;
extern lv_obj_t * ui_Batterie;
extern lv_obj_t * ui_Preis;
extern lv_obj_t * ui_PVRing;
extern lv_obj_t * ui_PV1;
extern lv_obj_t * ui_Batteriewert;
extern lv_obj_t * ui_Hausring;
extern lv_obj_t * ui_Verbrauch1;
extern lv_obj_t * ui_NetzRing;
extern lv_obj_t * ui_Netz1;
extern lv_obj_t * ui_AkkuRing;
extern lv_obj_t * ui_Akku1;
extern lv_obj_t * ui_Lichtseite;
extern lv_obj_t * ui_SofaLicht;
extern lv_obj_t * ui_SofaToggle;
extern lv_obj_t * ui_Label1;
extern lv_obj_t * ui_SofaSlider;
extern lv_obj_t * ui_TischLicht1;
extern lv_obj_t * ui_TischToggle;
extern lv_obj_t * ui_Label2;
extern lv_obj_t * ui_Tischslider;
extern lv_obj_t * ui_TabPage3;
extern lv_obj_t * ui_PVChart2;
// SCREEN: ui_OTA
void ui_OTA_screen_init(void);
extern lv_obj_t * ui_OTA;
extern lv_obj_t * ui_FirmwareBar;
extern lv_obj_t * ui_Label6;
// SCREEN: ui_Init
void ui_Init_screen_init(void);
extern lv_obj_t * ui_Init;
extern lv_obj_t * ui_Label7;
extern lv_obj_t * ui_Container7;
extern lv_obj_t * ui_Container4;
extern lv_obj_t * ui_Label12;
extern lv_obj_t * ui_Label8;
extern lv_obj_t * ui_Label9;
extern lv_obj_t * ui_Label10;
extern lv_obj_t * ui_Container5;
extern lv_obj_t * ui_IPLabel;
extern lv_obj_t * ui_Verbrauch2;
extern lv_obj_t * ui_Produktion2;
extern lv_obj_t * ui_Netz2;
extern lv_obj_t * ui____initial_actions0;


LV_IMG_DECLARE(ui_img_pv_png);    // assets/PV.png
LV_IMG_DECLARE(ui_img_haus_png);    // assets/Haus.png
LV_IMG_DECLARE(ui_img_netz_png);    // assets/Netz.png
LV_IMG_DECLARE(ui_img_akku_png);    // assets/Akku.png
LV_IMG_DECLARE(ui_img_pot_ver_line_png);    // assets/pot_ver_line.png
LV_IMG_DECLARE(ui_img_pot_ver_knob_png);    // assets/pot_ver_knob.png






void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
