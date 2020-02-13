package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Index extends SubsystemBase {
  private final CANSparkMax IndexBottom = RobotMap.IndexBottom;
  private final CANSparkMax IndexTop = RobotMap.IndexTop;


  public void IndexFill(double speed){
    /*if(laser sensor == flase){
      IndexBottom.set(-1);
    }
    else{
      IndexBottom.set(1);
    }*/
    IndexBottom.set(speed);
  }
  public void index(){
    if(Robot.m_oi.BailysJob.getRawButtonPressed(1) == true){
      IndexFill(1);
    }
    else {
      IndexBottom.set(0);
    }
  }

 public void IndexStop(){
  IndexBottom.set(0);
  IndexTop.set(0);
 }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
