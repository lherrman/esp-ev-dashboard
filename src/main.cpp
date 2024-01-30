#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <Wire.h>

#include <lvgl.h>
#include <TFT_eSPI.h>

#include <demos/lv_demos.h>
#include <touchscreen.hpp>


static const uint16_t screenWidth = 320;
static const uint16_t screenHeight = 480;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 6];

TFT_eSPI tft = TFT_eSPI(); 

bool state = false;

// define button callback function
static void btn_event_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    if (lv_event_get_code(e) == LV_EVENT_CLICKED)
    {
        if (state == false)
        {
            Serial.println("LED ON");
            state = true;
        }
        else
        {
            Serial.println("LED OFF");
            state = false;
        }
    }
    else if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED)
    {
        Serial.println("Toggled");
    }
}

void lv_example_btn(void)
{
    static lv_style_prop_t props[] = {
        LV_STYLE_TRANSFORM_WIDTH, LV_STYLE_TRANSFORM_HEIGHT, LV_STYLE_TEXT_LETTER_SPACE};

    /*Transition descriptor when going back to the default state.
      Add some delay to be sure the press transition is visible even if the press was very short*/
    static lv_style_transition_dsc_t transition_dsc_def;
    lv_style_transition_dsc_init(&transition_dsc_def, props, lv_anim_path_overshoot, 250, 100, NULL);

    /*Transition descriptor when going to pressed state.
      No delay, go to presses state immediately*/
    static lv_style_transition_dsc_t transition_dsc_pr;
    lv_style_transition_dsc_init(&transition_dsc_pr, props, lv_anim_path_ease_in_out, 250, 0, NULL);

    /*Add only the new transition to he default state*/
    static lv_style_t style_def;
    lv_style_init(&style_def);
    lv_style_set_transition(&style_def, &transition_dsc_def);

    /*Add the transition and some transformation to the presses state.*/
    static lv_style_t style_pr;
    lv_style_init(&style_pr);
    lv_style_set_transform_width(&style_pr, 5);
    lv_style_set_transform_height(&style_pr, -10);
    lv_style_set_text_letter_space(&style_pr, 10);
    lv_style_set_transition(&style_pr, &transition_dsc_pr);

    lv_obj_t *btn1 = lv_btn_create(lv_scr_act());
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -80);
    lv_obj_add_style(btn1, &style_pr, LV_STATE_PRESSED);
    lv_obj_add_style(btn1, &style_def, 0);

    lv_obj_t *label = lv_label_create(btn1);
    lv_label_set_text(label, "A/C");

    /*Init the style for the default state*/
    static lv_style_t style;
    lv_style_init(&style);

    lv_style_set_radius(&style, 3);

    lv_style_set_bg_opa(&style, LV_OPA_100);
    lv_style_set_bg_color(&style, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_color(&style, lv_palette_darken(LV_PALETTE_BLUE, 2));
    lv_style_set_bg_grad_dir(&style, LV_GRAD_DIR_VER);

    lv_style_set_border_opa(&style, LV_OPA_40);
    lv_style_set_border_width(&style, 2);
    lv_style_set_border_color(&style, lv_palette_main(LV_PALETTE_GREY));

    lv_style_set_shadow_width(&style, 8);
    lv_style_set_shadow_color(&style, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_shadow_ofs_y(&style, 8);

    lv_style_set_outline_opa(&style, LV_OPA_COVER);
    lv_style_set_outline_color(&style, lv_palette_main(LV_PALETTE_BLUE));

    lv_style_set_text_color(&style, lv_color_white());
    lv_style_set_pad_all(&style, 10);

    /*Init the pressed style*/
    static lv_style_t style_pr_2;
    lv_style_init(&style_pr_2);

    /*Ad a large outline when pressed*/
    lv_style_set_outline_width(&style_pr_2, 30);
    lv_style_set_outline_opa(&style_pr_2, LV_OPA_TRANSP);

    lv_style_set_translate_y(&style_pr_2, 5);
    lv_style_set_shadow_ofs_y(&style_pr_2, 3);
    lv_style_set_bg_color(&style_pr_2, lv_palette_darken(LV_PALETTE_BLUE, 2));
    lv_style_set_bg_grad_color(&style_pr_2, lv_palette_darken(LV_PALETTE_BLUE, 4));

    /*Add a transition to the the outline*/
    static lv_style_transition_dsc_t trans;
    static lv_style_prop_t props2[] = {LV_STYLE_OUTLINE_WIDTH, LV_STYLE_OUTLINE_OPA};
    lv_style_transition_dsc_init(&trans, props2, lv_anim_path_linear, 300, 0, NULL);

    lv_style_set_transition(&style_pr_2, &trans);

    lv_obj_t *btn2 = lv_btn_create(lv_scr_act());
    lv_obj_remove_style_all(btn2); /*Remove the style coming from the theme*/
    lv_obj_add_style(btn2, &style, 0);
    lv_obj_add_style(btn2, &style_pr_2, LV_STATE_PRESSED);
    lv_obj_set_size(btn2, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_center(btn2);

    lv_obj_t *label2 = lv_label_create(btn2);
    lv_label_set_text(label2, "Button");
    lv_obj_center(label2);



    //add callback
    lv_obj_add_event_cb(btn2, btn_event_cb, LV_EVENT_CLICKED, NULL);


}

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

void setup()
{
    Serial.begin(115200);
    delay(100);
    gt911_int_();

    lv_init();

    tft.begin();        
    tft.setRotation(0); 

    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 6);

    // Initialize and register a display driver
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    // Initialising the (virtual) input device driver
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = gt911_touchpad_read;
    lv_indev_drv_register(&indev_drv);


    lv_example_btn();

}

void loop()
{
    lv_timer_handler();
    delay(10);
}

