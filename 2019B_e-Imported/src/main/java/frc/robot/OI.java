/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public Joystick stick = new Joystick(0);

  public Joystick joy = new Joystick(1);

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


  public OI(){
   // A =  new JoystickButton(stick, 20);
    //A.whileHeld(new testcommand());

    UP = new JoystickButton(stick, 5);
    UP.whenPressed(new elevating());
    Down = new JoystickButton(stick, 6);
    Down.whenPressed(new elevating());

    in = new JoystickButton(joy, 9);
    in.whenPressed(new intaker());
    out = new JoystickButton(joy, 10);
    out.whenPressed(new manualRoller());

    armup = new JoystickButton(stick, 10);

    fast = new JoystickButton(stick, 2);
    fast.whenPressed(new fast());
    slow = new JoystickButton(stick, 1);
    slow.whenPressed(new slow());

    balllow = new JoystickButton(joy, 3);
    balllow.whenPressed(new TargetElevator());
    ballmid = new JoystickButton(joy, 2);
    ballmid.whenPressed(new TargetElevator());
    ballhigh = new JoystickButton(joy, 1);
    ballhigh.whenPressed(new TargetElevator());

    hatchlow = new JoystickButton(joy, 6);
    hatchlow.whenPressed(new TargetElevator());
    hatchmid = new JoystickButton(joy, 5);
    hatchmid.whenPressed(new TargetElevator());
    hatchhigh = new JoystickButton(joy, 4);
    hatchhigh.whenPressed(new TargetElevator());

    ManUp = new JoystickButton(joy, 7);
    ManUp.whenPressed(new manualRoller());
    ManDown = new JoystickButton(joy, 8);
    ManDown.whenPressed(new manualRoller());

    InitProb = new JoystickButton(joy, 11);
    InitProb.whenPressed(new theprober());

    PinchBtn = new JoystickButton(joy, 12);
    PinchBtn.whenPressed(new manualRoller());
    
    Ybtn = new JoystickButton(stick, 4);
    Ybtn.whenPressed(new climbFunction());

  }
/*
  Button testButton = new JoystickButton(stick, 1);

  public OI(){

  testButton.whenPressed(new testcommand());
  }*/

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
