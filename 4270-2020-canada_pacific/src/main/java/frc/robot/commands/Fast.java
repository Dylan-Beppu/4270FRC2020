
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Shifter;

public class Fast extends CommandBase {
    private final Shifter kShifter;

  public Fast(Shifter subsystem) {
    kShifter = subsystem;
    addRequirements(kShifter);
  }
  
    // Called just before this Command runs the first time
    @Override
    public void initialize() {
      //sets to low gear
      kShifter.isfast = false;
      kShifter.slow();
      kShifter.realese1();
    //kShifter.IntakeUp();
    }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    kShifter.shifter ();
    kShifter.Intakeppos();
    kShifter.realese();
    //kShifter.turrthood(); 
    
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
