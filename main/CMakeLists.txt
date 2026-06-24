idf_component_register(
                         SRCS "main.c"
                              "LCD_Driver/Vernon_ST7789T/Vernon_ST7789T.c"
                              "LCD_Driver/ST7789.c"
                              "LVGL_Driver/LVGL_Driver.c"
                              "LVGL_UI/LVGL_Heatmap.c"
                              "DMOS5031/dmos5031.c"
                              "DMOS5031/dmos5031_frame.c"
                              "DMOS5031/dragonfly.c"
                              "BAT_Driver/BAT_Driver.c"

                         INCLUDE_DIRS
                              "./LCD_Driver/Vernon_ST7789T"
                              "./LCD_Driver"
                              "./LVGL_Driver"
                              "./LVGL_UI"
                              "./DMOS5031"
                              "./BAT_Driver"
                              "."
                       )
