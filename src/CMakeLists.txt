SET(SOURCES components/ui_comp_batteriewert.c
    components/ui_comp.c
    screens/ui_PV.c
    screens/ui_OTA.c
    screens/ui_Init.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.cpp
    images/ui_img_pv_png.c
    images/ui_img_haus_png.c
    images/ui_img_netz_png.c
    images/ui_img_akku_png.c
    images/ui_img_pot_ver_line_png.c
    images/ui_img_pot_ver_knob_png.c)

add_library(ui ${SOURCES})
