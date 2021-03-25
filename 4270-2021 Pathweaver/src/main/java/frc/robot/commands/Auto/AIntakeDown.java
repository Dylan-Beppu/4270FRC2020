package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shifter;

//for help with the new way https://docs.wpilib.org/en/latest/docs/software/commandbased/commands.html
//if the can bus isent compleat, neos wornt work!!! But talonfx might :)
public class AIntakeDown extends CommandBase {
  private final Shifter kShifter;
  private final Intake kIntake;
  private final Drivetrain kDrivetDrive;

  public AIntakeDown() {
    kShifter = Robot.kShifter;
    kIntake = Robot.kIntake;
    kDrivetDrive = Robot.kDrivetrain;

    addRequirements(kIntake);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kShifter);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    kShifter.IntakeDown(); 
    kIntake.intakeInAuto();
    kDrivetDrive.setOutputVolts(-2, -2);

    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    kDrivetDrive.setOutputVolts(0.0, 0.0);
    kIntake.intakeStop();
    kShifter.IntakeUp();
  }
}
