package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.Shooter;
import frc.robot.subsystems.*;

public class Auto1 extends SequentialCommandGroup {
  public Auto1(Drivetrain drive, Intake intake /*, Limelight vision, Shooter shooter*/) {
    addCommands(
      new Auto1Path(drive)
      //new ATurret(Robot.kTurret),
      //new AIntakeDown(Robot.kShifter).alongWith(new AIntakeIN(Robot.kIntake))
      //TODO: find out how to put wait commands in code wether it be in this file or in each commmand file      
      );//,
      //change this to commands
      //Robot.kShifter.hoodup().alongwith(Robot.kShifter.fast()));

  }


}