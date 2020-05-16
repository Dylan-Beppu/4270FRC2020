package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Hang extends SubsystemBase {
  private final CANSparkMax Endgame = RobotMap.Endgame;

  /**
   * Creates a new ExampleSubsystem.
   */
  public Hang() {

  }
  public void hangg(){
    if(Robot.m_oi.OtherBTN2 == true){
      Endgame.set(1);
    }
    else{
      Endgame.set(0);
    }

  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
