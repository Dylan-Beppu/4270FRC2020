package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Turret;

public class Shooter extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret m_subsystem;

   /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    public Shooter(Turret subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    Robot.kTurret.areweinrange();
    Robot.kTurret.camencreset();
    //Robot.kProbe.probingfunction();
    Robot.kTurret.toggleon();
    Robot.kTurret.shootshoot();
    Robot.kTurret.track();
    Robot.kTurret.camPosReset();
   
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
