package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.Fast;

public class Shifter extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final DoubleSolenoid speedsole = RobotMap.shifter;

  public boolean isfast;

  public void fast(){
    speedsole.set(DoubleSolenoid.Value.kForward);
  }

  public void slow(){
    speedsole.set(DoubleSolenoid.Value.kReverse);
  }

}
