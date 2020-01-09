/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.fast;
//import frc.robot.commands.slow;

/**
 * Add your docs here.
 */
public class shifter extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final DoubleSolenoid speedsole = RobotMap.shifter;

  public boolean isfast;


  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new fast());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void fast(){
    speedsole.set(DoubleSolenoid.Value.kForward);
  }

  public void slow(){
    speedsole.set(DoubleSolenoid.Value.kReverse);
  }

}
