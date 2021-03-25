package frc.robot.commands;

//import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.Turret;

//for help with the new way https://docs.wpilib.org/en/latest/docs/software/commandbased/commands.html
//if the can bus isent compleat, neos wornt work!!! But talonfx might :)
public class Shooter extends CommandBase {
  private final Turret kTurret;
    
  public Shooter(Turret subsystem) {
    kTurret = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kTurret);
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
     Robot.kTurret.areweinrange();
    ////Robot.kTurret.camencreset();
    Robot.kTurret.toggleon();
    ////Robot.kTurret.shootshoot();
    Robot.kTurret.track();
    ////Robot.kTurret.camPosReset();
    //Robot.kTurret.basic();
   
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
  }
}
