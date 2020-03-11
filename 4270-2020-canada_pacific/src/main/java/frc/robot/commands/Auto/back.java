package frc.robot.commands.Auto;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;

//for help with the new way https://docs.wpilib.org/en/latest/docs/software/commandbased/commands.html
//if the can bus isent compleat, neos wornt work!!! But talonfx might :)
public class back extends CommandBase {
  private final Drivetrain kdrive;
    
  public back(Drivetrain subsystem) {
    kdrive = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kdrive);
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    //    m_climber.climb(50);
    
    
 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    kdrive.autoBSpeed(-0.3);
    RobotMap.rightdrive1.set(0.3);
    RobotMap.rightdrive2.set(0.3);
    RobotMap.leftdrive1.set(-0.3);
    RobotMap.leftdrive2.set(-0.3);
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
  }
}
