package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Index extends SubsystemBase {
  private final CANSparkMax IndexBottom = RobotMap.IndexBottom;
  private final CANSparkMax LeftIntake = RobotMap.LeftIntake;
  private final DigitalInput BeamBrakeTop = RobotMap.BeamBrakeTop;
  private final DigitalInput BeamBrakeBottom = RobotMap.BeamBrakeBotom;
  
  

  //think in 3 positions, top, mid, low
  public void IndexFill(){
    //top = none, mid = ball, low
    if(Robot.m_oi.BailysJob.getRawAxis(3) != 0 && Robot.kTurret.togglebtn == false){
      if(BeamBrakeTop.get() == false && BeamBrakeBottom.get() == true ){
        IndexBottom.set(-1);
        LeftIntake.set(-1);
      }
      //top = ball, mid = none
      else if(BeamBrakeTop.get() == true && BeamBrakeBottom.get() == false ){
        IndexBottom.set(0);
        LeftIntake.set(-1);
      }
      //top = ball, mid = ball
      else if(BeamBrakeTop.get() == true && BeamBrakeBottom.get() == true){
        IndexBottom.set(0);
        LeftIntake.set(0);
      }
    }
    else if(Robot.m_oi.BailysJob.getRawAxis(3) != 0 && Robot.kTurret.togglebtn == false){
      RobotMap.CenterIntake.set(0.5);
      IndexBottom.set(-0.7);
    }
    else{
      IndexBottom.set(0);
      LeftIntake.set(0);
    }
  }

  public void index(){
    if(Robot.m_oi.BailysJob.getRawAxis(2) != 0){
      LeftIntake.set(-1);
      //IndexFill(1);
    }
    else {
      //IndexBottom.set(0);
      LeftIntake.set(0);
    }
  }

 public void IndexStop(){
  //IndexBottom.set(0);
  RobotMap.LeftIntake.set(0);
 }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
