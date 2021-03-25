package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoTest1 extends SequentialCommandGroup {


  public AutoTest1() {
   //  this.drive = drive;
     //runAuto();
     addCommands(new ParallelDeadlineGroup(
      new AShoot().withTimeout(1.5),
      new ATurret(true, false)
    ));
  
      // addCommands(new AShoot(0));
      //change this to commands
      //Robot.kShifter.hoodup().alongwith(Robot.kShifter.fast()));

  }
}