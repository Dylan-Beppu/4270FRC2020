package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Turret;

public class Shooter extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_subsystem;

    @param subsystem 

    public Shooter(Turret) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Turret):
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.kvision.areweinrange();
    Robot.kvision.camencreset();
    //Robot.kProbe.probingfunction();
    Robot.kvision.toggleon();
    Robot.kvision.shootshoot();
    Robot.kvision.track();
    Robot.kvision.camPosReset();
   
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //Robot.kProbe.switchExtension();
    Robot.kvision.spinStop();
    Robot.kvision.unblindMe();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
