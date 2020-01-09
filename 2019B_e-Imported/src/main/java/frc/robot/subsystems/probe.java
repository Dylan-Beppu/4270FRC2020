/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
//import frc.robot.commands.theprober;

/**
 * Add your docs here.
 */
public class probe extends Subsystem {
  private final DoubleSolenoid prober = RobotMap.Probe;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public boolean isExtended = false;

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new theprober());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void in(){
    prober.set(Value.kForward);
  }
  public void out(){
    prober.set(Value.kReverse);
  }
  public void switchExtension(){
    if(isExtended == false){
      isExtended = true;
    }
    else if(isExtended == true){
      isExtended = false;
    }
  }
  public void probingfunction(){
    if(isExtended == false){
      prober.set(Value.kForward);
    }
    else if(isExtended == true){
      prober.set(Value.kReverse);
    }
  }

}
