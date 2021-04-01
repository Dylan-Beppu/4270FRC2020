package frc.robot.commands.AutoRouteans;
import frc.robot.commands.Auto.*;
//import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Robot;
@SuppressWarnings("unused")
public class Auto3 extends SequentialCommandGroup {
  Drivetrain kDrivetrain = Robot.kDrivetrain;
  // Use addRequirements() here to declare subsystem dependencies.
  

  public Auto3() {
    addRequirements(kDrivetrain);

    addCommands(
      //new ParallelDeadlineGroup(
      //  new AIntakeDown1().withTimeout(2)
      //),
    new ParallelDeadlineGroup(
     // new ADrive(),
      kDrivetrain.FollowPath("Unnamed.wpilib.json")
    )
    //new ParallelDeadlineGroup(
    //  new AFillup(),
    //  new AIntakeDown()
    //)
    );
  }
}