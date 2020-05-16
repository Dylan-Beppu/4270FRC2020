package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.Robot;
//import frc.robot.commands.*;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  //public Joystick BailysJob = new Joystick(0);
  Joystick DriversJob = new Joystick(3);
  public Joystick BtnPanle = new Joystick(1);

  //controler
  //dpad
  public boolean DpadUp = DriversJob.getRawButton(1);
  public boolean DpadDown = DriversJob.getRawButton(2);
  public boolean DpadLeft = DriversJob.getRawButton(3);
  public boolean DpadRight = DriversJob.getRawButton(4);
  //Gp is gamepad
  public boolean GpUp = DriversJob.getRawButton(100);
  public boolean GpDown = DriversJob.getRawButton(3);
  public boolean GpLeft = DriversJob.getRawButton(100);
  public boolean GpRight = DriversJob.getRawButton(2);
  //joysticks
  public boolean LjoyBtn = DriversJob.getRawButton(7);
  public double LjoyX = DriversJob.getRawAxis(0);
  public double LjoyY = DriversJob.getRawAxis(1);
  public boolean RjoyBtn = DriversJob.getRawButton(7);
  public double RjoyX = DriversJob.getRawAxis(2);
  public double RjoyY = DriversJob.getRawAxis(3);

  //triggers (note trigger may be either buttons or joysticks)
  public boolean L1 = DriversJob.getRawButton(7);
  public boolean L2 = DriversJob.getRawButton(5);
  //public double L2 = DriversJob.getRawAxis(5);
  public boolean R1 = DriversJob.getRawButton(6);
  public boolean R2 = DriversJob.getRawButton(8);
  //public double R2 = DriversJob.getRawAxis(8);
  //other buttons
  public boolean OtherBTN1 = DriversJob.getRawButton(9);
  public boolean OtherBTN2 = DriversJob.getRawButton(10);
  //public boolean OtherBTN3 = DriversJob.getRawButton(11);
  //public boolean OtherBTN4 = DriversJob.getRawButton(12);


  //Button panle
  public boolean BtnPanle1 = DriversJob.getRawButton(1);
  public boolean BtnPanle2 = DriversJob.getRawButton(2);
  public boolean BtnPanle3 = DriversJob.getRawButton(3);
  public boolean BtnPanle4 = DriversJob.getRawButton(4);
  public boolean BtnPanle5 = DriversJob.getRawButton(5);
  public boolean BtnPanle6 = DriversJob.getRawButton(6);
  public boolean BtnPanle7 = DriversJob.getRawButton(7);
  public boolean BtnPanle8 = DriversJob.getRawButton(8);



  //JoystickButton BtnPanle1;

  public OI(){
    //JoystickButton HIDin1 = new JoystickButton(BailysJob, 1);

   // A =  new JoystickButton(stick, 20);
    //A.whileHeld(new testcommand());
  
  }
}