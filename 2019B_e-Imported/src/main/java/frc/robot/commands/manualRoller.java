/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class manualRoller extends Command {
  public manualRoller() {
    requires(Robot.kElevator);
    //requires(Robot.kDrivetrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    //setTimeout(2);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.ManUp.get() == true && Robot.m_oi.ManDown.get() == false){
   //   Robot.kroller.manintakeup();
   Robot.kElevator.targetIntake(4, .4);
  
    }
    else if(Robot.m_oi.ManDown.get() == true && Robot.m_oi.ManUp.get() == false){
  // Robot.kroller.manintakedown();
 Robot.kElevator.targetIntake(32, .4);

    }
    else if(Robot.m_oi.out.get() == true){
      Robot.kElevator.manintakeup();
    }
    else{
      Robot.kElevator.manintakeoff();
    }

    if(Robot.m_oi.PinchBtn.get() == true){
      RobotMap.wristmotor.getEncoder().setPosition(0);
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
    Robot.kroller.manintakeoff();

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
