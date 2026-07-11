SET(SOURCES components/ui_comp_softwarupdate.c
    components/ui_comp.c
    screens/ui_MainConfigScreen.c
    screens/ui_MemoryEraseValidation.c
    screens/ui_OTAUpdate.c
    screens/ui_Wifi.c
    screens/ui_ManualWifi.c
    screens/ui_InclinometerGraph.c
    screens/ui_Inclinometer.c
    screens/ui_InclinometerNew.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_rollpajero2_reduced_png.c
    images/ui_img_rollreduced_png.c
    fonts/ui_font_Nabla_Big.c
    fonts/ui_font_HeadingDegree.c
    fonts/ui_font_SmallerEmblemaOne.c)

add_library(ui ${SOURCES})
