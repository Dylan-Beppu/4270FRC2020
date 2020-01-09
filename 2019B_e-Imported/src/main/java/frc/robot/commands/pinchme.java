/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class pinchme extends Command {
  public pinchme() {
    requires(Robot.kpinch);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   /* if(Robot.m_oi.PinchBtn.get() == true  && Robot.m_oi.joy.getRawAxis(1) == 0){
      Robot.kpinch.grab();
    }
    else if(Robot.m_oi.PinchBtn.get() == false && Robot.m_oi.joy.getRawAxis(1) == -1){
      Robot.kpinch.release();
    }*/
    if(Robot.kpinch.lionholler == true){
      Robot.kpinch.release();
    }
    else if(Robot.kpinch.lionholler == false){
      Robot.kpinch.grab();
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
