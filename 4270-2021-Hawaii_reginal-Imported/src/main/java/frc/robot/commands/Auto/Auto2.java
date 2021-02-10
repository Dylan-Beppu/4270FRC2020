package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class Auto2 extends SequentialCommandGroup {


  public Auto2() {
   //  this.drive = drive;
     //runAuto();
     
    addCommands(new ParallelDeadlineGroup(
      new AFillup(),
      new AIntakeDown()
    ));
    addCommands(new ParallelDeadlineGroup(
    new AShoot().withTimeout(3),  
    new ATurret(true, true)
    ));

  }
}