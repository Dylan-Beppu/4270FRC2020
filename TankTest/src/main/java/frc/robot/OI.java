package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;

import java.awt.Button;

import edu.wpi.first.wpilibj.GenericHID;

public class OI {
    /*
    Add your joysticks and buttons here
    */
    private Joystick primaryJoystick = new Joystick(0);
    //private Joystick arcadeStick = new Joystick(1);
//    private XboxController testController = new XboxController(2);
    
    public OI() {
        public Joystick PrimaryJoystick = primaryJoystick;
      
    public Joystick getPrimaryJoystick() {
        return primaryJoystick;
    }
}
