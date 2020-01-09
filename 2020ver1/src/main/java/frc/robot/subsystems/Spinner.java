package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.RobotMap;
import frc.robot.commands.Spinning;
import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Spinner extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final CANSparkMax BB8 = RobotMap.R2D2;
  

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new Spinning());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void ManSpiner(double ColorSpeed){
    BB8.set(ColorSpeed);
  }
  public void AutoSpin(double Ammount){
    //Ammount2 = Ammount*2;
  }
  public void AutoColor(){

  }
}
