/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class intaker extends Command {
  public intaker() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.kIntake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.in.get() == true && Robot.m_oi.stick.getRawAxis(3) == 0){
      Robot.kIntake.suck();
    }
    else if(Robot.m_oi.stick.getRawAxis(3) != 0 && Robot.m_oi.in.get() == false){
      Robot.kIntake.eject();
    }
    else{
      Robot.kIntake.chill();
    }
    /*     if(Robot.m_oi.in.get() == true && Robot.m_oi.out.get() == false){
      Robot.kIntake.suck();
    }
    else if(Robot.m_oi.out.get() == true && Robot.m_oi.in.get() == false){
      Robot.kIntake.eject();
    }
    else{
      Robot.kIntake.chill();
    }*/
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.kIntake.chill();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
