package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  public final CANSparkMax Intake = RobotMap.Intake;
  //private final Shifter kShifter = Robot.kShifter;
  
  public void intakeMan() {
    if(Robot.m_oi.PanIntakeDown.get() == true){
      Intake.set(-0.75); 
       
      //kShifter.IntakeDown();
    }
    else{
      Intake.set(0);
      //kShifter.IntakeUp();
    }
  }



 public void intakeStop(){
    Intake.set(0);
 }
 public void intakeInAuto(){
  Intake.set(-0.75); 
 }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
