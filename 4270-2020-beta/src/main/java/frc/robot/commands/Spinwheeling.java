package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Robot;
//import frc.robot.RobotMap;
import frc.robot.subsystems.Spinwheel;;

public class Spinwheeling extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Spinwheel kSpinwheel;

   /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    public Spinwheeling(Spinwheel subsystem) {
    kSpinwheel = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kSpinwheel);
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    //Robot.kSpinwheel.colorDectect();
   
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
