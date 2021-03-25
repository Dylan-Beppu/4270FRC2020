package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

//for help with the new way https://docs.wpilib.org/en/latest/docs/software/commandbased/commands.html
//if the can bus isent compleat, neos wornt work!!! But talonfx might :)
public class ADrive extends CommandBase {
  private final Drivetrain kDrivetDrive;
    
  public ADrive() {
    kDrivetDrive = Robot.kDrivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kDrivetDrive);
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    //    m_climber.climb(50);
   // Robot.kIndex.IndexFill();
   kDrivetDrive.followSet(true);
    
 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    //kTurret.turretAuto();
    kDrivetDrive.Camfollow();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    kDrivetDrive.followSet(false);

    kDrivetDrive.setOutputVolts(0.0, 0.0);
    //Robot.kProbe.switchExtension();
    //Robot.kTurret.spinStop();
    //Robot.kTurret.unblindMe();
  }
}
