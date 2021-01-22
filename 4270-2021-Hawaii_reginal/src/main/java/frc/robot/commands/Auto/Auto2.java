package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

public class Auto2 extends SequentialCommandGroup {
  private boolean firstRun = true;
  private boolean secondrun = true;
  private boolean thirdrun = true;
  private double startTime2;

  private double  startTime1;
  private double startTime;
  private Drivetrain drive;
  public Auto2(Drivetrain drive /*, Limelight vision, Shooter shooter*/) {
     this.drive = drive;
     runAuto();
     
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
    if (thirdrun = true) {
      startTime2 = Timer.getFPGATimestamp();
      thirdrun = false;
    }
    while(Timer.getFPGATimestamp() - startTime2 < 4){
      
    }

    if (firstRun = true) {
      startTime = Timer.getFPGATimestamp();
      firstRun = false;
    }
    /*while(Timer.getFPGATimestamp() - startTime < 0.8){
      drive.setOutputVolts(-4, -4);
      Robot.kTurret.turretAuto();
      Robot.kTurret.togglebtn = true;
    }*/
    drive.setOutputVolts(0.0, 0.0);

    if (secondrun = true) {
      startTime1 = Timer.getFPGATimestamp();
      secondrun = false;
    }
    while(Timer.getFPGATimestamp() - startTime1 < 5){
      Robot.kIndex.Aindexup();
      Robot.kTurret.turretAuto();
      RobotMap.Topin.set(-1); 
    } 
    Robot.kTurret.togglebtn = false;
    Robot.kTurret.spinStop();
    Robot.kTurret.unblindMe();
    Robot.kIndex.IndexStop();
    RobotMap.FlyboiR.set(0);
    RobotMap.FlyboiL.set(0);
    RobotMap.Topin.set(0); 


    

/*
    if (Timer.getFPGATimestamp() - startTime < 0.1){
      drive.setOutputVolts(1.5, 1.5);
    }
    else{
      drive.setOutputVolts(0.0, 0.0); 
    } */

   }
   public void voolts(){
    if (Timer.getFPGATimestamp() - startTime < 0.1){
      drive.setOutputVolts(3, 3);
    }
    else{
      drive.setOutputVolts(0.0, 0.0); 
    } 
   }

}