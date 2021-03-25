package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

public class Auto1 extends SequentialCommandGroup {
  boolean firstRun = true;
  boolean secondrun = true;
  boolean thirdrun = true;
  private double startTime2;
  private double  startTime1;
  private double startTime;
  private Drivetrain drive;
  public Auto1(Drivetrain drive /*, Limelight vision, Shooter shooter*/) {
     this.drive = drive;
     runAuto();
  }

  public void runAuto() {
    if (firstRun = true) {
      startTime = Timer.getFPGATimestamp();
      firstRun = false;
    }
    while(Timer.getFPGATimestamp() - startTime < 1){
      drive.setOutputVolts(4, 4);
      Robot.kTurret.turretAuto(true,true);
      Robot.kTurret.togglebtn = true;
    }

    

    if (secondrun = true) {
      startTime1 = Timer.getFPGATimestamp();
      secondrun = false;
    }
    while(Timer.getFPGATimestamp() - startTime1 < 5){
      RobotMap.Topin.set(-1);
      //Robot.kIndex.Aindexup();
      Robot.kTurret.turretAuto(true,true);
    } 
    Robot.kTurret.togglebtn = false;
    Robot.kTurret.spinStop();
    Robot.kTurret.unblindMe();
    Robot.kIndex.IndexStop();
    RobotMap.FlyboiR.set(0);
    RobotMap.FlyboiL.set(0);
    RobotMap.Topin.set(0); 

    if (thirdrun = true) {
      startTime2 = Timer.getFPGATimestamp();
      thirdrun = false;
    }
    while(Timer.getFPGATimestamp() - startTime2 < 2){
      
      drive.setOutputVolts(4, 4);
    }
    drive.setOutputVolts(0.0, 0.0);
   }
}