package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

//for help with the new way https://docs.wpilib.org/en/latest/docs/software/commandbased/commands.html
//if the can bus isent compleat, neos wornt work!!! But talonfx might :)
public class AindexStop extends CommandBase {
  private final Index kIndex;
    
  public AindexStop(Index subsystem) {
    kIndex = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kIndex);
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    //    m_climber.climb(50);
   // Robot.kIndex.IndexFill();
    kIndex.IndexStop();
    
 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    //kTurret.turretAuto();

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    //Robot.kProbe.switchExtension();
    //Robot.kTurret.spinStop();
    //Robot.kTurret.unblindMe();
  }
}
