/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class elevating extends Command {
  public elevating() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.kElevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.Down.get() == true && Robot.m_oi.UP.get() == false){
      Robot.kElevator.Down(.4);
    }
    else if(Robot.m_oi.Down.get() == false && Robot.m_oi.UP.get() == true){
      Robot.kElevator.Up(.6);
    }
    else{
      Robot.kElevator.Up(0);
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
    Robot.kElevator.Up(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
