/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <math.h>
#include "lvgl.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define START_SCREEN (uint8_t) 0
#define GAME_SCREEN  (uint8_t) 1
#define END_SCREEN   (uint8_t) 2

#define TICK_FOR_UPDATE 20

#define BUTTON_PRESSED (uint8_t) 1
#define BUTTON_UNPRESSED (uint8_t) 0

#define CLOUD_START_X (lv_coord_t) 120
#define CLOUD_1_START_Y (lv_coord_t) 25
#define CLOUD_2_START_Y (lv_coord_t) 20
#define CLOUD_3_START_Y (lv_coord_t) 30
#define CLOUD_END_X (lv_coord_t) -20
#define CLOUD_END_Y (lv_coord_t) 25
#define CLOUD_STEP_X (lv_coord_t) 10
#define CLOUD_STEP_Y (lv_coord_t) 0

#define BARRIER_START_X (lv_coord_t) 130
#define BARRIER_START_Y (lv_coord_t) 83
#define BARRIER_END_X (lv_coord_t) -20
#define BARRIER_END_Y (lv_coord_t) 83
#define BARRIER_STEP_X (lv_coord_t) 20
#define BARRIER_STEP_Y (lv_coord_t) 0

#define POTITA_START_X (lv_coord_t) 10
#define POTITA_START_Y (lv_coord_t) 70
#define POTITA_END_X (lv_coord_t) 10
#define POTITA_END_Y (lv_coord_t) 70
#define POTITA_STEP_X (lv_coord_t) 0
#define POTITA_STEP_Y (lv_coord_t) 40

#define AREA_JUMP_MAX_X (lv_coord_t) 60
#define AREA_JUMP_MIN_X (lv_coord_t) 10

#define GROUND_X (lv_coord_t) 0
#define GROUND_Y (lv_coord_t) 103

static const char *TAG = "example";

// LVGL image declare
LV_IMG_DECLARE(initial_screen)
LV_IMG_DECLARE(potita_step_1)
LV_IMG_DECLARE(potita_step_2)
LV_IMG_DECLARE(potita_end_game)
LV_IMG_DECLARE(barrier_1)
LV_IMG_DECLARE(ground)
LV_IMG_DECLARE(cloud_1)
LV_IMG_DECLARE(cloud_2)
LV_IMG_DECLARE(cloud_3)

typedef struct{
    lv_obj_t *img;
    lv_coord_t start_x;
    lv_coord_t start_y;
    lv_coord_t end_x;
    lv_coord_t end_y;
    uint8_t step_x;
    uint8_t step_y;
    uint8_t count; 
} moving_object_t;

typedef struct{
    uint32_t tick_animation;
    uint8_t current_screen;
    uint8_t state_button_enter;
    uint8_t state_button_control; 
    uint8_t flag_press_enter_button;
    uint8_t flag_press_control_button; 
    uint8_t flag_toggle_animation;
    uint8_t flag_jump;
    uint8_t count_cloud_img;
    uint8_t points;        
    moving_object_t potita;
    moving_object_t barrier;
    moving_object_t cloud;
} animation_t;

static animation_t game;

typedef struct {
    lv_obj_t *scr;
    int count_val;
} my_timer_context_t;

static lv_obj_t *dis;

static lv_obj_t *img_initial_screen;
static lv_obj_t *text_title;
static lv_obj_t *img_ground;
static lv_obj_t *text_start;
static lv_obj_t *text_points;

// Buttons
extern xQueueHandle enter_button_queue;
extern xQueueHandle control_button_queue;

extern lv_disp_t *disp;

void  horizontal_move(lv_obj_t *scr, moving_object_t *obj, const void * source_img)
{
    if(obj->count == 0)     
    {
         // Create obj
        obj->img = lv_img_create(scr);
        lv_img_set_src(obj->img, source_img);
        lv_obj_set_pos(obj->img, obj->start_x, obj->start_y); 
        obj->count++;                
    }
    else {
        lv_coord_t next_x = obj->start_x - (obj->count*obj->step_x);

        if(next_x >= obj->end_x) {
            lv_obj_set_x(obj->img, next_x);
            obj->count++;
        }
        else {
            lv_obj_del(obj->img);
            obj->count = 0; 
        }
    }  
}

void animation_game(lv_obj_t *scr, animation_t *game_str)
{
    lv_coord_t barrier_x; 

    game_str->tick_animation++;

    // Check button
    if(xQueueReceiveFromISR(enter_button_queue, &game_str->state_button_enter, pdMS_TO_TICKS(10))) {
        game_str->flag_press_enter_button = BUTTON_PRESSED;
    }   

    if(xQueueReceiveFromISR(control_button_queue, &game_str->state_button_control, pdMS_TO_TICKS(10))) {
        game_str->flag_press_control_button = BUTTON_PRESSED;
        game_str->flag_jump = 1;
    }   

    if(game_str->flag_press_enter_button == BUTTON_PRESSED)
    {
        if(game_str->current_screen == START_SCREEN) {

            game_str->current_screen = GAME_SCREEN;

            lv_obj_del(img_initial_screen);
            lv_obj_del(text_title);
            lv_obj_del(text_start);

            img_ground = lv_img_create(scr);
            lv_img_set_src(img_ground, &ground);
            lv_obj_set_pos(img_ground, GROUND_X, GROUND_Y); 

            game_str->potita.img = lv_img_create(scr);
            lv_img_set_src(game_str->potita.img, &potita_step_1);
            lv_obj_set_pos(game_str->potita.img, game_str->potita.start_x, game_str->potita.start_y); 

            text_points = lv_label_create(lv_scr_act());
            lv_label_set_long_mode(text_points, LV_LABEL_LONG_DOT);
            lv_obj_set_pos(text_points, 10, 110); 
            lv_label_set_text_fmt(text_points, "%d", game_str->points);  
            lv_obj_align(text_points, LV_ALIGN_BOTTOM_RIGHT, 0, 0);                
        }
        else if(game_str->current_screen == END_SCREEN) {

            game_str->current_screen = GAME_SCREEN;

            lv_obj_del(text_title);
            lv_obj_del(text_start); 
            lv_obj_del(game_str->cloud.img); 

            game_str->potita.count = 0;
            game_str->barrier.count = 0;
            game_str->cloud.count = 0;
            game_str->count_cloud_img = 0;
            game_str->flag_jump = 0;  
            game_str->points = 0;    

            lv_label_set_text_fmt(text_points, "%d", game_str->points);    
        }
        game_str->flag_press_enter_button = BUTTON_UNPRESSED;
    }

    // Update animation of the game every 600 ms
    if(game_str->tick_animation >= TICK_FOR_UPDATE) {

        if(game_str->current_screen == START_SCREEN) {

            if(game_str->flag_toggle_animation) {
                lv_label_set_text(text_start, "#ff0000 START#");
                game_str->flag_toggle_animation = 0;
            }
            else {
                lv_label_set_text(text_start, "#000000 START#");   
                game_str->flag_toggle_animation = 1;     
            }
        }
        else if(game_str->current_screen == END_SCREEN) {

            if(game_str->flag_toggle_animation) {
                lv_label_set_text(text_start, "#ff0000 AGAIN#");
                game_str->flag_toggle_animation = 0;
            }
            else {
                lv_label_set_text(text_start, "#000000 AGAIN#");   
                game_str->flag_toggle_animation = 1;     
            }
        }
        else if(game_str->current_screen == GAME_SCREEN) {

            // Potita jump
            if(game_str->flag_jump == 1) {

                if(game_str->potita.count == 0) {

                    lv_coord_t next_y = POTITA_START_Y - POTITA_STEP_Y;
                    lv_obj_set_y(game_str->potita.img, next_y);
                    game_str->potita.count++;
                }
                else if(game_str->potita.count == 1) {

                    // lv_coord_t next_y = POTITA_START_Y - (POTITA_STEP_Y*2);
                    // lv_obj_set_y(game_str->potita.img, 30);
                    game_str->potita.count++;
                }
                else {
                    lv_obj_set_y(game_str->potita.img, POTITA_START_Y);
                    game_str->potita.count = 0;
                    game_str->flag_jump = 0;
                }
            }

            // Animation of Potita walking 
            if(game_str->flag_jump == 0)
            {
                if(game_str->flag_toggle_animation) {
                    lv_img_set_src(game_str->potita.img, &potita_step_1);
                    game_str->flag_toggle_animation = 0;
                }
                else {
                    lv_img_set_src(game_str->potita.img, &potita_step_2);  
                    game_str->flag_toggle_animation = 1;
                    }
            }

            // Barrier
            horizontal_move(scr, &game_str->barrier, &barrier_1); 

            // Check on the success of jumping the barrier
            if(game_str->barrier.count == 0) {
                game_str->points++;
                lv_label_set_text_fmt(text_points, "%d", game_str->points); 
            }

            // Clouds
            // Choose image of cloud
            if(game_str->count_cloud_img == 0) {
                
                game_str->cloud.start_y = CLOUD_1_START_Y;
                horizontal_move(scr, &game_str->cloud, &cloud_1);
            }
            else if (game_str->count_cloud_img == 1) {

                game_str->cloud.start_y = CLOUD_2_START_Y;
                horizontal_move(scr, &game_str->cloud, &cloud_2);
            }
            else {
                game_str->cloud.start_y = CLOUD_3_START_Y;
                horizontal_move(scr, &game_str->cloud, &cloud_3);
            }

            if(game_str->cloud.count == 0) {                   

                if(game_str->count_cloud_img < 2) {

                    game_str->count_cloud_img++;
                }
                else {
                    game_str->count_cloud_img = 0;
                }               
            }

            lv_obj_move_foreground(game_str->potita.img);
            
            // Checking for collision with a barrier
            barrier_x = lv_obj_get_x(game_str->barrier.img);
            if((barrier_x < AREA_JUMP_MAX_X) && (barrier_x > AREA_JUMP_MIN_X) && (game_str->flag_jump == 0)) {
                
                // Collision
                game_str->current_screen = END_SCREEN; 

                lv_obj_del(game_str->barrier.img);
                lv_img_set_src(game_str->potita.img, &potita_end_game);

                text_start = lv_label_create(lv_scr_act());
                lv_label_set_long_mode(text_start, LV_LABEL_LONG_DOT);
                lv_label_set_recolor(text_start, true); /*Enable re-coloring by commands in the text*/
                lv_obj_set_pos(text_start, 10, 110); 
                lv_label_set_text(text_start, "AGAIN");
                lv_obj_align(text_start, LV_ALIGN_BOTTOM_LEFT, 0, 0);

                static lv_style_t style_text;
                text_title = lv_label_create(lv_scr_act());
                lv_style_init(&style_text);
                lv_style_set_text_font(&style_text, &lv_font_montserrat_20);
                lv_obj_add_style(text_title, &style_text, LV_STATE_DEFAULT);
                lv_obj_align(text_title, LV_ALIGN_TOP_MID, 0, 5);
                lv_obj_set_size(text_title, 128, 20);
                lv_label_set_text(text_title, "GAME OVER");
                lv_label_set_long_mode(text_title, LV_LABEL_LONG_WRAP);                 
            }
        }
        game_str->tick_animation = 0;
    }
}

void prepare_animation_lvgl(lv_obj_t *scr)
{
    // init str for animation game
    game.tick_animation = 0;
    game.current_screen = START_SCREEN;
    game.state_button_enter = 0;
    game.state_button_control = 0; 
    game.flag_press_enter_button = 0;
    game.flag_press_control_button = 0; 
    game.flag_toggle_animation = 0;
    game.flag_jump = 0; 
    game.count_cloud_img = 0;
    game.points = 0;

    game.potita.start_x = POTITA_START_X;
    game.potita.start_y = POTITA_START_Y;
    game.potita.end_x = POTITA_END_X;
    game.potita.end_y = POTITA_END_Y;    
    game.potita.step_x = POTITA_STEP_X;    
    game.potita.step_y = POTITA_STEP_Y; 
    game.potita.count = 0;

    game.cloud.start_x = CLOUD_START_X;
    game.cloud.start_y = CLOUD_1_START_Y;
    game.cloud.end_x = CLOUD_END_X;
    game.cloud.end_y = CLOUD_END_Y;    
    game.cloud.step_x = CLOUD_STEP_X;    
    game.cloud.step_y = CLOUD_STEP_Y; 
    game.cloud.count = 0;

    game.barrier.start_x = BARRIER_START_X;
    game.barrier.start_y = BARRIER_START_Y;
    game.barrier.end_x = BARRIER_END_X;
    game.barrier.end_y = BARRIER_END_Y;    
    game.barrier.step_x = BARRIER_STEP_X;    
    game.barrier.step_y = BARRIER_STEP_Y; 
    game.barrier.count = 0;

    //Background
    lv_color_t color = lv_color_hex(0xffffff);
    lv_disp_set_bg_color(disp, color);

    // Create image
    img_initial_screen = lv_img_create(scr);
    lv_img_set_src(img_initial_screen, &initial_screen);
    lv_obj_center(img_initial_screen);

    // Create text
    static lv_style_t style_text;
    text_title = lv_label_create(lv_scr_act());
    lv_style_init(&style_text);
    lv_style_set_text_font(&style_text, &lv_font_montserrat_20);
    lv_obj_add_style(text_title, &style_text, LV_STATE_DEFAULT);
    lv_obj_align(text_title, LV_ALIGN_TOP_MID, 0, 5);
    lv_obj_set_size(text_title, 120, 20);
    lv_label_set_text(text_title, "POTITA GAME");
    lv_label_set_long_mode(text_title, LV_LABEL_LONG_SCROLL_CIRCULAR);

    // Create text start
    text_start = lv_label_create(lv_scr_act());
    lv_label_set_long_mode(text_start, LV_LABEL_LONG_DOT);
    lv_label_set_recolor(text_start, true); /*Enable re-coloring by commands in the text*/
    lv_obj_set_pos(text_start, 10, 110); 
    lv_label_set_text(text_start, "START");
    lv_obj_align(text_start, LV_ALIGN_BOTTOM_LEFT, 0, 0);
}

void animation_lvgl(lv_obj_t *scr)
{
    animation_game(scr, &game);
}

    
    
