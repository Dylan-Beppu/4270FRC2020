package frc.robot.commands.Auto;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Intake;

//for help with the new way https://docs.wpilib.org/en/latest/docs/software/commandbased/commands.html
//if the can bus isent compleat, neos wornt work!!! But talonfx might :)
public class AIntakeDown extends CommandBase {
  private final Shifter kShifter;
    
  public AIntakeDown(Shifter subsystem) {
    kShifter = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kShifter);
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  kShifter.IntakeDown(); 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    
  }
}
