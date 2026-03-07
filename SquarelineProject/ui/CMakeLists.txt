SET(SOURCES components/ui_comp_softwarupdate.c
    components/ui_comp.c
    screens/ui_InclinometerGraph.c
    screens/ui_Inclinometer.c
    screens/ui_Wifi.c
    screens/ui_OTAUpdate.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    fonts/ui_font_Nabla_Big.c
    fonts/ui_font_HeadingDegree.c
    fonts/ui_font_SmallerEmblemaOne.c)

add_library(ui ${SOURCES})
