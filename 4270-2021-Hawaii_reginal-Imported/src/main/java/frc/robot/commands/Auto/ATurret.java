package frc.robot.commands.Auto;
//import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Turret;

public class ATurret extends CommandBase {
  private final Turret kTurret;
  boolean Amode;
  boolean Astate;
  public ATurret(boolean state, boolean mode) {
    Astate = state;
    Amode = mode;

    kTurret = Robot.kTurret;
    addRequirements(kTurret);    
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
    kTurret.turretAuto(Astate, Amode);
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
    kTurret.turretAuto(false, false);
  }
}
