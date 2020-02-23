package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Fast;

public class Shifter extends SubsystemBase {
  private final DoubleSolenoid speedsole = RobotMap.shifter;
  private final DoubleSolenoid Armsole = RobotMap.arm;

  public boolean isfast;

  //drive gearig
  public void fast(){
    speedsole.set(DoubleSolenoid.Value.kForward);
  }

  public void slow(){
    speedsole.set(DoubleSolenoid.Value.kReverse);
  }

  public void IntakeDown(){
    Armsole.set(DoubleSolenoid.Value.kReverse);
  }

  public void IntakeUp(){
    Armsole.set(DoubleSolenoid.Value.kForward);
  }
  public void Intakeppos(){
  if(Robot.m_oi.BailysJob.getRawButtonPressed(5) == true){
    IntakeDown();
  }
  else{
    IntakeUp();
  }
}

}
