package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.Robot;
//import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public Joystick Driver = new Joystick(0);

  public Joystick BtnPanle = new Joystick(1);
  // Joystick configuration (Switch pro controler)
  /*
   * public JoystickButton DriTargTog = new JoystickButton(Driver, 6); public
   * JoystickButton DriTargShoot = new JoystickButton(Driver, 8); public
   * JoystickButton DriGearUp = new JoystickButton(Driver, 3); public
   * JoystickButton DriGearDown = new JoystickButton(Driver, 2); public
   * JoystickButton DriClimbRels = new JoystickButton(Driver, 9); public
   * JoystickButton DriClimbClm = new JoystickButton(Driver, 10);
   */

  // xbox controles
  public JoystickButton DriTargTog = new JoystickButton(Driver, 6);
  public JoystickButton DriTargShoot = new JoystickButton(Driver, 5);
  public JoystickButton DriGearUp = new JoystickButton(Driver, 2);
  public JoystickButton DriGearDown = new JoystickButton(Driver, 1);
  public JoystickButton DriClimbRels = new JoystickButton(Driver, 17);
  public JoystickButton DriClimbClm = new JoystickButton(Driver, 18);
  /*
   * public JoystickButton Joybtn7 = new JoystickButton(Driver, 7); public
   * JoystickButton Joybtn8 = new JoystickButton(Driver, 8); public JoystickButton
   * Joybtn9 = new JoystickButton(Driver, 9); public JoystickButton Joybtn10 = new
   * JoystickButton(Driver, 10); public JoystickButton Joybtn11 = new
   * JoystickButton(Driver, 11); public JoystickButton Joybtn12 = new
   * JoystickButton(Driver, 12); public JoystickButton Joybtn13 = new
   * JoystickButton(Driver, 13); public JoystickButton Joybtn14 = new
   * JoystickButton(Driver, 14); public JoystickButton Joybtn15 = new
   * JoystickButton(Driver, 15); public JoystickButton Joybtn16 = new
   * JoystickButton(Driver, 16); public JoystickButton Joybtn17 = new
   * JoystickButton(Driver, 17); public JoystickButton Joybtn18 = new
   * JoystickButton(Driver, 18);
   */

  // Buton Panle configeration
  /*
   * public JoystickButton PanTargetTogle = new JoystickButton(Driver, 18); public
   * JoystickButton PanShoot = new JoystickButton(Driver, 18); public
   * JoystickButton PanBleh = new JoystickButton(Driver, 18); public
   * JoystickButton PanIntakeDown = new JoystickButton(Driver, 18); public
   * JoystickButton PanIndex = new JoystickButton(Driver, 18); public
   * JoystickButton PanOut = new JoystickButton(Driver, 18);
   */

  public JoystickButton PanTargetTogle = new JoystickButton(BtnPanle, 1);
  public JoystickButton PanShoot = new JoystickButton(BtnPanle, 2);
  public JoystickButton PanBleh = new JoystickButton(BtnPanle, 3);
  public JoystickButton PanIntakeDown = new JoystickButton(BtnPanle, 4);
  public JoystickButton PanIndex = new JoystickButton(BtnPanle, 5);
  public JoystickButton PanOut = new JoystickButton(BtnPanle, 6);

  /*
   * public JoystickButton Panbtn7 = new JoystickButton(BtnPanle, 7); public
   * JoystickButton Panbtn8 = new JoystickButton(BtnPanle, 8);
   */

  public OI() {
    // Panbtn1 = new JoystickButton(BtnPanle, 1);
    // out = new JoystickButton(joy, 10);
    // out.whenPressed(new manualRoller());
    // A = new JoystickButton(stick, 20);
    // A.whileHeld(new testcommand());

  }
}