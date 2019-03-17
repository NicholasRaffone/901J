#include "main.h"
#include "display/lv_conf.h"
#include "config.hpp"
#include <string>

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

 static lv_res_t btn_rel_action(lv_obj_t * btn)
 {
    lv_obj_t * label1 = lv_obj_get_child(btn, NULL); /*The label is the only child*/
    if(blueSide == true){
      blueSide = false;
      lv_label_set_text(label1, "Red");
    } else{
      blueSide = true;
      lv_label_set_text(label1, "Blue");

    }
    return LV_RES_OK;
 }
 static lv_res_t btn_rel_action_two(lv_obj_t * btn)
 {

   lv_obj_t * label1 = lv_obj_get_child(btn, NULL); /*The label is the only child*/
       if(farSide == true){
         farSide = false;
         lv_label_set_text(label1, "Close");
       } else{
         farSide = true;
         lv_label_set_text(label1, "Far");
       }
   return LV_RES_OK;
 }

 static lv_res_t display_auton(lv_obj_t * btn)
 {
   lv_obj_t * mbox1 = lv_mbox_create(lv_scr_act(), NULL);
   lv_obj_set_size(mbox1, 450, 500);
   lv_obj_align(mbox1, NULL, LV_ALIGN_CENTER, 10, 10); /*Align to the center*/

   std::string color;
   std::string side;

   if (blueSide == true){
    color = "Blue";
  } else{
    color= "Red";
  }

  if(farSide == true){
    side="Far" ;
  }else{
   side="Close";
  }
  std::string concat = color + " " + side + " selected";

  const char* c_data = concat.c_str( );
   lv_mbox_set_text(mbox1, c_data);
   lv_mbox_start_auto_close(mbox1, 2500);
   //lv_mbox_stop_auto_close(mbox1);
   return LV_RES_OK;
 }

void initialize() {/*Create a button*/

    lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);         /*Create a button on the currently loaded screen*/
    lv_obj_t * label = lv_label_create(btn1, NULL);
    lv_label_set_text(label, "Red");
		lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, btn_rel_action); /*Set function to be called when the button is released*/
    lv_obj_align(btn1, label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 20);  /*Align below the label*/

    /*Create a label on the button (the 'label' variable can be reused)*/
    //lv_label_set_text(label1, "Red");

    /*Copy the previous button*/
    lv_obj_t * btn2 = lv_btn_create(lv_scr_act(), btn1);        /*Second parameter is an object to copy*/
    label = lv_label_create(btn2, NULL);
    lv_label_set_text(label, "Close");
    lv_btn_set_action(btn2, LV_BTN_ACTION_CLICK, btn_rel_action_two);
    lv_obj_align(btn2, btn1, LV_ALIGN_OUT_RIGHT_MID, 35, 0);    /*Align next to the prev. button.*/

    lv_obj_t * btn3 = lv_btn_create(lv_scr_act(), btn1);
    lv_btn_set_action(btn3, LV_BTN_ACTION_CLICK, display_auton);
    lv_obj_align(btn3, btn2, LV_ALIGN_OUT_RIGHT_TOP, 10,-60);
    label = lv_label_create(btn3, NULL);
    lv_label_set_text(label, "Display \nAuton");

    lv_obj_t * mbox2 = lv_mbox_create(lv_scr_act(), NULL);
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
