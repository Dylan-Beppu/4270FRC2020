package frc.robot.commands.Auto;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
//import frc.robot.RobotMap;
//import frc.robot.subsystems.Drivetrain;

public class AutoTest2 extends SequentialCommandGroup {
  public AutoTest2() {
   //  runAuto();
  }

  public void runAuto() {
    while (true){
      //Robot.kTurret.blindMe();
      Robot.kTurret.unblindMe();

    }
  }


}