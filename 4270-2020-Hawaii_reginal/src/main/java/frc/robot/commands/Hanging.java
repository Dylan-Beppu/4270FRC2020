package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Hang;

public class Hanging extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Hang kHang;

    public Hanging(Hang subsystem) {
    kHang = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kHang);
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    Robot.kHang.hangg();
    // Robot.kHang.realese();
    
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
