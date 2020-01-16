/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class shooter extends Command {
  public shooter() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.kFlywheel);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.kFlywheel.flyspeed(100);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      //up, down
    if(Robot.m_oi.primaryController.getRawButtonPressed(1) == true && Robot.m_oi.primaryController.getRawButtonPressed(2) == false){
        //Robot.kFlywheel.Down(.4);
      }
      //down, up
      else if(Robot.m_oi.primaryController.getRawButtonPressed(2) == false && Robot.m_oi.primaryController.getRawButtonPressed(2) == true){
        //Robot.kFlywheel.Up(.6);
      }
      else{
        //Robot.kFlywheel.Up(0);
      }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //Robot.kFlywheel.Spindown();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
