package frc.robot.commands.Auto;
//import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class ATurret extends CommandBase {
  private final Turret kTurret;
  private final Index kIndex;

  public ATurret() {
    kTurret = Robot.kTurret;
    kIndex = Robot.kIndex;
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    //kTurret.togglebtn = true;
     //    m_climber.climb(50);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    kTurret.turretAuto(true);
    kIndex.Aindexup();
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
    Robot.kTurret.spinStop();
    Robot.kTurret.unblindMe();
    kTurret.turretAuto(false);
    kIndex.IndexStop();

  }
}
