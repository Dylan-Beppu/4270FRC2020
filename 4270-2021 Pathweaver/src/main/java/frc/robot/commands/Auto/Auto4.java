package frc.robot.commands.Auto;

//import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Robot;
public class Auto4 extends SequentialCommandGroup {
  Drivetrain kDrivetrain = Robot.kDrivetrain;
  // Use addRequirements() here to declare subsystem dependencies.
  

  public Auto4() {
    addRequirements(kDrivetrain);

    addCommands(
      //new ParallelDeadlineGroup(
      //  new AIntakeDown1().withTimeout(2)
      //),
    new ParallelDeadlineGroup(
     // new ADrive(),
      kDrivetrain.FollowPath("6ft.wpilib.json")
    )
    //new ParallelDeadlineGroup(
    //  new AFillup(),
    //  new AIntakeDown()
    //)
    );
  }
}