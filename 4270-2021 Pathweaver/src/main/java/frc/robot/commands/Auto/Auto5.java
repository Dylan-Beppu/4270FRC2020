package frc.robot.commands.Auto;

//import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Robot;
public class Auto5 extends SequentialCommandGroup {
  Drivetrain kDrivetrain = Robot.kDrivetrain;
  // Use addRequirements() here to declare subsystem dependencies.
  

  public Auto5() {
    addRequirements(kDrivetrain);

    addCommands(
      //new ParallelDeadlineGroup(
      //  new AIntakeDown1().withTimeout(2)
      //),
    new ParallelDeadlineGroup(
     // new ADrive(),
      kDrivetrain.FollowPath("180.wpilib.json")
    )
    //new ParallelDeadlineGroup(
    //  new AFillup(),
    //  new AIntakeDown()
    //)
    );
  }
}