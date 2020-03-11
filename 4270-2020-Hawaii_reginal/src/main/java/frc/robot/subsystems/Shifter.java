package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
//import frc.robot.commands.Fast;

public class Shifter extends SubsystemBase {
  private final static DoubleSolenoid speedsole = RobotMap.shifter;
  private final static DoubleSolenoid Armsole = RobotMap.arm;
  private final static DoubleSolenoid hood = RobotMap.hood;
  private final static DoubleSolenoid releasH = RobotMap.releasH;

  public boolean isfast;

  // drive gearig
  public static void fast() {
    speedsole.set(DoubleSolenoid.Value.kForward);
  }

  public void slow(){
    speedsole.set(DoubleSolenoid.Value.kReverse);
  }

  public void hoodup(){

    hood.set(DoubleSolenoid.Value.kForward);
  }

  public void hooddown(){
    hood.set(DoubleSolenoid.Value.kReverse);
  }

  public void IntakeDown(){
    Armsole.set(DoubleSolenoid.Value.kReverse);
  }

  public void IntakeUp(){
    Armsole.set(DoubleSolenoid.Value.kForward);
  }
  public void relasehang(){
    releasH.set(DoubleSolenoid.Value.kReverse);
  }

  public void Intakeppos(){
  if(Robot.m_oi.BtnPanle.getRawButton(1) == true){
    IntakeDown();
  }
  else{
    IntakeUp();
  }
  }
  public void realese1(){
    releasH.set(DoubleSolenoid.Value.kForward);
  }
  public void realese(){
    if(Robot.m_oi.BailysJob.getRawButton(9) == true){
      Robot.kShifter.relasehang();
    }
  }
    public void turrthood(){
      if(Robot.m_oi.BtnPanle.getRawButton(6) == true){
        hoodup();
      }
      else if(Robot.m_oi.BtnPanle.getRawButton(8) == true){
        hooddown();
      }
    }
    public void shifter(){
      if(Robot.m_oi.BailysJob.getRawButton(2) == true){
        fast();
    }
    else if(Robot.m_oi.BailysJob.getRawButton(3) == true){
      slow();
    }
  }
}
