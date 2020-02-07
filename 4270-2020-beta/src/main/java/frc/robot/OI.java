/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public Joystick BailysJob = new Joystick(0);

  public Joystick SomeOneElce = new Joystick(1);

  //public JoystickButton A;

  public JoystickButton UP;

  public JoystickButton Down;

  public JoystickButton in;

  public JoystickButton out;

  public JoystickButton armup;

  public JoystickButton armdown;

  public JoystickButton fast;
  public JoystickButton slow;

  public JoystickButton balllow;
  public JoystickButton ballmid;
  public JoystickButton ballhigh;

  public JoystickButton hatchlow;
  public JoystickButton hatchmid;
  public JoystickButton hatchhigh;

  public JoystickButton ManUp;
  public JoystickButton ManDown;

  public JoystickButton InitProb;
  public JoystickButton PinchBtn;
  public JoystickButton Ybtn;
  public JoystickButton test;


  public OI(){
   // A =  new JoystickButton(stick, 20);
    //A.whileHeld(new testcommand());
  }
}