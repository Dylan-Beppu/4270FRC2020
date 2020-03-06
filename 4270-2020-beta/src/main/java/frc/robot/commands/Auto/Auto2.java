package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.Shooter;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Auto2 extends SequentialCommandGroup {
  private boolean firstRun = true;
  private double startTime;
  private Drivetrain drive;
  public Auto2(Drivetrain drive /*, Limelight vision, Shooter shooter*/) {
     this.drive = drive;
    // addCommands(
    //   //new ATurret(Robot.kTurret),
    //   //new AShoot(Robot.kIndex),
    //   //new WaitCommand(3),
    //   new back(Robot.kDrivetrain),
    //   //new ATurret(Robot.kTurret)
    //   new WaitCommand(3),
    //   new drivestop(Robot.kDrivetrain)

    //   //new ATurret(Robot.kTurret),
    //   //new AIntakeDown(Robot.kShifter).alongWith(new AIntakeIN(Robot.kIntake))
    //   //TODO: find out how to put wait commands in code wether it be in this file or in each commmand file      
    //   );//,
      //change this to commands
      //Robot.kShifter.hoodup().alongwith(Robot.kShifter.fast()));

  }

  public void runAuto() {
    if (firstRun) {
      startTime = Timer.getFPGATimestamp();
      firstRun = false;
    }
    if (Timer.getFPGATimestamp() - startTime < 2.0)
      drive.setOutputVolts(12.0, 12.0); 
    else drive.setOutputVolts(0.0, 0.0); 

   }


}