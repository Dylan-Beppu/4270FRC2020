package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
//import frc.robot.OI;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  private final CANSparkMax Intake = RobotMap.Intake;
  private final Shifter kShifter = Robot.kShifter;
  
  public void intakeMan(int button) {
    if(Robot.m_oi.BailysJob.getRawButtonPressed(button) == true){
      Intake.set(-1);
      kShifter.IntakeDown();
    }
    else{
      Intake.set(0);
      kShifter.IntakeUp();
    }
  }

  public void intakeAuto(double speed, boolean Down) {
    if(Down == true){
      Intake.set(-1);
      kShifter.IntakeDown();
    }
    else if(Down == false){
      Intake.set(0);
      kShifter.IntakeUp();
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
