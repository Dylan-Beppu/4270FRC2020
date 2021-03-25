package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class Auto3 extends SequentialCommandGroup {


  public Auto3() {
    addCommands(
      new ParallelDeadlineGroup(
        new AIntakeDown1().withTimeout(2)
      ),
    new ParallelDeadlineGroup(
      new ADrive()
    ),
    new ParallelDeadlineGroup(
      new AFillup(),
      new AIntakeDown()
    )
    );
  }
}