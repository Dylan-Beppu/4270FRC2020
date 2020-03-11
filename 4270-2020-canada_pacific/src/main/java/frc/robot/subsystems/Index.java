package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Auto.AShoot;

public class Index extends SubsystemBase {
  private final CANSparkMax IndexBottom = RobotMap.IndexBottom;
  private final CANSparkMax centerIntake = RobotMap.CenterIntake;
  private final DigitalInput BeamBrakeTop = RobotMap.BeamBrakeTop;
  private final DigitalInput BeamBrakeBottom = RobotMap.BeamBrakeBotom;
  public boolean ISFill;

  /*public boolean CheckISfill() {
    if(BeamBrakeTop.get() == false && BeamBrakeBottom.get() == false){
      ISFill = true;
    } 
    return ISFill;
  }*/

  public void Aindexfill(){
    while(ISFill = false){
    if(BeamBrakeTop.get() == true && BeamBrakeBottom.get() == false ){
      IndexBottom.set(-1);
      centerIntake.set(0.5);
      RobotMap.LeftIntake.set(-0.5);
    }
    //top = ball, mid = none
    else if(BeamBrakeTop.get() == false && BeamBrakeBottom.get() == true ){
      IndexBottom.set(0);
      centerIntake.set(0.5);
      RobotMap.LeftIntake.set(-0.5);
    }
    //top = ball, mid = ball
    else if(BeamBrakeTop.get() == false && BeamBrakeBottom.get() == false){
      IndexBottom.set(0);
      centerIntake.set(0);
      RobotMap.LeftIntake.set(0);
    }
    else if(BeamBrakeTop.get() == true && BeamBrakeBottom.get() == true){
      IndexBottom.set(-1);
      centerIntake.set(0.5);
      RobotMap.LeftIntake.set(-0.5);
    }
  }
  IndexBottom.set(0);
  centerIntake.set(0);
  RobotMap.LeftIntake.set(0);
  //Robot.kFillup.isFinished();
}
  //think in 3 positions, top, mid, low
  public void IndexFill(){
    //top = none, mid = ball, low
    if(Robot.m_oi.BtnPanle.getRawButton(4) == true && Robot.kTurret.togglebtn == false){
      //RobotMap.LeftIntake.set(-0.5);
      if(BeamBrakeTop.get() == true && BeamBrakeBottom.get() == false ){
        IndexBottom.set(-1);
        centerIntake.set(0.5);
        RobotMap.LeftIntake.set(-0.5);
      }
      //top = ball, mid = none
      else if(BeamBrakeTop.get() == false && BeamBrakeBottom.get() == true ){
        IndexBottom.set(0);
        centerIntake.set(0.5);
        RobotMap.LeftIntake.set(-0.5);
      }
      //top = ball, mid = ball
      else if(BeamBrakeTop.get() == false && BeamBrakeBottom.get() == false){
        IndexBottom.set(0);
        centerIntake.set(0);
        RobotMap.LeftIntake.set(0);
      }
      else if(BeamBrakeTop.get() == true && BeamBrakeBottom.get() == true){
        IndexBottom.set(-1);
        centerIntake.set(0.5);
        RobotMap.LeftIntake.set(-0.5);

      }
    }
    else if(Robot.m_oi.BailysJob.getRawButton(8) == true && Robot.kTurret.togglebtn == true){
      RobotMap.CenterIntake.set(0.5);
      IndexBottom.set(-1);
      RobotMap.LeftIntake.set(-0.5);
      RobotMap.Topin.set(-1);
    }
    
    else if(Robot.m_oi.BtnPanle.getRawButton(6) == true){
      RobotMap.CenterIntake.set(-0.5);
      IndexBottom.set(1);
      RobotMap.LeftIntake.set(0.5);

    }
    else if(Robot.m_oi.BtnPanle.getRawButton(2) == true){
      RobotMap.CenterIntake.set(0.5);
      IndexBottom.set(-1);
      RobotMap.LeftIntake.set(-0.5);
      RobotMap.Topin.set(-1);
    }
    else{
      IndexBottom.set(0);
      centerIntake.set(0);
      RobotMap.LeftIntake.set(0);
      RobotMap.Topin.set(0);
    }
  }

  public void index(){
    if(Robot.m_oi.BailysJob.getRawAxis(2) != 0){
      centerIntake.set(-1);
      //IndexFill(1);
    }
    else {
      //IndexBottom.set(0);
      centerIntake.set(0);
    }
  }
  /*
  ** upup down stop
  */
  public void Aindexup(){
    RobotMap.CenterIntake.set(0.5);
      IndexBottom.set(-1);
      RobotMap.LeftIntake.set(-0.5);
  }
 public void IndexStop(){
  //IndexBottom.set(0);
  RobotMap.CenterIntake.set(0);
      IndexBottom.set(0);
      RobotMap.LeftIntake.set(0);
 }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
