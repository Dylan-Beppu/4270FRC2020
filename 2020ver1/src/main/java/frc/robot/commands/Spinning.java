package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class Spinning extends Command {
  public Spinning() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.kSpinner);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      //up, down
    if(Robot.m_oi.secondaryController.getRawButtonPressed(1) == true && Robot.m_oi.secondaryController.getRawButtonPressed(2) == false){
        Robot.kSpinner.ManSpiner(.4);
      }
      //down, up
      else if(Robot.m_oi.secondaryController.getRawButtonPressed(2) == false && Robot.m_oi.secondaryController.getRawButtonPressed(2) == true){
        Robot.kSpinner.ManSpiner(-.4);
      }
      else{
        Robot.kSpinner.ManSpiner(0);
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
