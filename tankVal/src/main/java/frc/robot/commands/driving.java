/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.OI;

/**
 * An example command.  You can replace me with your own command.
 */
public class driving extends Command {
  public driving() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.kDrivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Robot.kDrivetrain.shoot();
    Robot.kDrivetrain.topspeed();
    Robot.kDrivetrain.bottomspeed();
    Robot.kDrivetrain.tank();
    Robot.kDrivetrain.outputToSmartDashboard();
    //Robot.kdrive.findingtiks();
      
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
      Robot.kDrivetrain.driveStraight(0);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
