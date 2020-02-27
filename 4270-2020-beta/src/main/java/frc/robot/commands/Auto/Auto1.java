package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class Auto1 extends SequentialCommandGroup {
  public Auto1(Drivetrain drive, Intake intake /*, Limelight vision, Shooter shooter*/) {
    addCommands(
      new Auto1Path(drive));//,
      //change this to commands
      //Robot.kShifter.hoodup().alongwith(Robot.kShifter.fast()));

  }


}