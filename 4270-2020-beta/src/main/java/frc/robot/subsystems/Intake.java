package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
//import frc.robot.OI;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
    private final CANSparkMax IntakeL = RobotMap.LeftIntake;
    private final CANSparkMax IntakeR = RobotMap.RightIntake;
    private final CANSparkMax IntakeC = RobotMap.CenterIntake;
  public void intakeMan(double speed) {
    if(Robot.m_oi.BailysJob.getRawButtonPressed(1) == true){
      IntakeL.set(speed);
      IntakeR.set(speed);
      IntakeC.set(-speed);
    }
    else{
      IntakeL.set(0);
      IntakeR.set(0);
      IntakeC.set(0);
    }
  }

  public void intakeAuto(double speed) {
    IntakeL.set(speed);
    IntakeR.set(speed);
    IntakeC.set(-speed);
  }

 public void intakeStop(){
    IntakeL.set(0);
    IntakeC.set(0);
    IntakeR.set(0);
 }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
