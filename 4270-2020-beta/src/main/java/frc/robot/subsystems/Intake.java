package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.jni.CANSparkMaxJNI;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
//import frc.robot.OI;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  public final CANSparkMax Intake = RobotMap.Intake;
  private final Shifter kShifter = Robot.kShifter;
  
  public void intakeMan() {
    if(Robot.m_oi.BtnPanle.getRawButton(1) == true){
      Intake.set(-0.8); 
       
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
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
