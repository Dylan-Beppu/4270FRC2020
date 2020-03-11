package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.buttons.Button;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.Robot;
//import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public Joystick BailysJob = new Joystick(0);

  public Joystick BtnPanle = new Joystick(1);
  //JoystickButton BtnPanle1;

  public OI(){
   // A =  new JoystickButton(stick, 20);
    //A.whileHeld(new testcommand());
  
  }
}