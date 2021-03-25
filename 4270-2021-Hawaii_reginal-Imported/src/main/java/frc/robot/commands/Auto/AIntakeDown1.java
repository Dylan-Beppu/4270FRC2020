package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shifter;

//for help with the new way https://docs.wpilib.org/en/latest/docs/software/commandbased/commands.html
//if the can bus isent compleat, neos wornt work!!! But talonfx might :)
public class AIntakeDown1 extends CommandBase {
  private final Shifter kShifter;
  private final Intake kIntake;

  public AIntakeDown1() {
    kShifter = Robot.kShifter;
    kIntake = Robot.kIntake;
    addRequirements(kIntake);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kShifter);
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    kShifter.IntakeDown(); 
    //kIntake.intakeInAuto();
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    //kIntake.intakeStop();
    //kShifter.IntakeUp();
  }
}
