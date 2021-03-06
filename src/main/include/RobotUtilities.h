/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

/**
 * Enumeration used for the Joystick Input IDs. Each value of the enumeration
 * corresponds with a joystick button or axis. Think of the joystick button or
 * axis you want to read from, and use this enumeration to increase future
 * readibility.
 */
enum JoystickInputs 
{ 
  x_axis = 0, y_axis = 1, z_axis = 2, w_axis = 3,
  x_button = 1, a_button = 2, b_button = 3, y_button = 4,
  left_bumper = 5, right_bumper = 6, left_trigger = 7, right_trigger = 8,
  back_button = 9, start_button = 10, l_trigger = 11, r_trigger = 12
};