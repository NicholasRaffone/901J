#include "main.h"
#include "display/lv_conf.h"
#include "config.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

lv_obj_t * label1;
lv_obj_t * label2;

 static lv_res_t btn_rel_action(lv_obj_t * btn)
 {
    autonColor++;
    //label1 = lv_label_create(btn, NULL);
    //lv_obj_align(btn, label1, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 20);
    if(autonColor %2 == 0){
      lv_label_set_text(label1, "Red");
    } else{
      lv_label_set_text(label1, "Blue");
    }
    return LV_RES_OK;
 }
 static lv_res_t btn_rel_action_two(lv_obj_t * btn)
 {
   autonSide++;
   //label2 = lv_label_create(btn, NULL);
   //lv_obj_align(btn, label2, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 20);
   if(autonColor %2 == 0){
     lv_label_set_text(label2, "Close");
   } else{
     lv_label_set_text(label2, "Far");
   }
   return LV_RES_OK;
 }

void initialize() {/*Create a button*/

    lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);         /*Create a button on the currently loaded screen*/
		label1 = lv_label_create(btn1, NULL);
		lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, btn_rel_action); /*Set function to be called when the button is released*/
    lv_obj_align(btn1, label1, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 20);  /*Align below the label*/

    /*Create a label on the button (the 'label' variable can be reused)*/
    lv_label_set_text(label1, "Red");

    /*Copy the previous button*/
    lv_obj_t * btn2 = lv_btn_create(lv_scr_act(), btn1);        /*Second parameter is an object to copy*/
    lv_btn_set_action(btn2, LV_BTN_ACTION_CLICK, btn_rel_action_two);
    lv_obj_align(btn2, btn1, LV_ALIGN_OUT_RIGHT_MID, 50, 0);    /*Align next to the prev. button.*/

    /*Create a label on the button*/
    label2 = lv_label_create(btn2, NULL);
    lv_label_set_text(label2, "Blue");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
