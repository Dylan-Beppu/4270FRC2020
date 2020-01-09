package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.RobotMap;
import frc.robot.commands.shooter;
//import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Flywheel extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final CANSparkMax Flyboi = RobotMap.Flyboy;

  CANEncoder encoder;
  CANPIDController controllerboi;

  //Constants constants = Constants.getConstants();
  public double shooterP = 0.0011;
  public double shooterI = 0;
  public double shooterD = 4;
  public double shooterF = 0.00017;
  
  double setpoint = 0;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new shooter());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void Shooter() {
    Flyboi.set(0);

    Flyboi.setIdleMode(IdleMode.kCoast);
    
    encoder = Flyboi.getEncoder();
    controllerboi = Flyboi.getPIDController();
    controllerboi.setFeedbackDevice(encoder);
    stop();
    updateConstants();
}

  public void updateConstants() {
    controllerboi.setOutputRange(-1, 0);
    controllerboi.setP(shooterP);
    controllerboi.setI(shooterI);
    controllerboi.setD(shooterD);
    controllerboi.setFF(shooterF);
  }

  public void stop(){
    controllerboi.setReference(0, ControlType.kDutyCycle);
  }

  public void set(double setpoint) {
    SmartDashboard.putNumber("shooterSetpoint", Math.abs(setpoint));

    //FireLog.log("shooterSetpoint", Math.abs(setpoint));
    controllerboi.setReference(setpoint, ControlType.kVelocity);
  }
  
  /*//fly wheel start/stop (hopfully pid)
  public void flyspeed(double speed){

  }

  //aiming fly wheel
  public void highgoal(){

  }
  public void lowgoal(){

  }*/
}
