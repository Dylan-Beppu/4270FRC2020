/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
//import frc.robot.commands.climbFunction;

/**
 * Add your docs here.
 */
public class Ultra extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final Ultrasonic ultra = RobotMap.ultra;
  private final DoubleSolenoid front = RobotMap.FrontClimb;
  private final DoubleSolenoid back = RobotMap.BackClimb;

public boolean kdone = false;

  @Override
  public void initDefaultCommand() {
  //  setDefaultCommand(new climbFunction());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void timetoclimb() { ///old one; the 2nd hab
    back.set(Value.kReverse); //out
    Timer.delay(2);
    while(kdone != true){
    if(ultra.getRangeInches() >= 4 ){
     // back.set(Value.kForward);
     // front.set(Value.kReverse);
      Robot.kDrivetrain.drivestraight(.2);
    }
    else if(ultra.getRangeInches() < 4){
      Robot.kDrivetrain.drivestraight(0.02);
      back.set(Value.kForward); //in
      Timer.delay(0.5);
      front.set(Value.kForward); //out
      Timer.delay(2);
      Robot.kDrivetrain.drivestraight(.3);
      Timer.delay(1);
      front.set(Value.kReverse);
      kdone = true;
    }
  }
    kdone=false;
  }

  public void thirdhab() {
    Robot.kElevator.targetIntake(30, .4);
    Robot.kElevator.complexElevate(78, 1);
    front.set(Value.kForward);
    Robot.kElevator.targetIntake(4, .8);
    Robot.kDrivetrain.justdrive(2);
    Robot.kElevator.targetIntake(20, .8);
    Robot.kIntake.speedsuck(.5);
    Robot.kDrivetrain.justdrive(1);
    while(Robot.kElevator.Edoneyet != true || Robot.kElevator.Idoneyet != true){
    Robot.kElevator.compoundClimb(2, .3, 72, .3);
    }
   Robot.kIntake.timedSuck(2, .5);
   Robot.kDrivetrain.justdrive(2);
   front.set(Value.kReverse);
  }

  public void secondhab() {
    Robot.kElevator.targetIntake(30, .4);
    Robot.kElevator.complexElevate(78, 1);
    front.set(Value.kForward);
    Robot.kElevator.targetIntake(4, .8);
    Robot.kDrivetrain.justdrive(2);
    Robot.kElevator.targetIntake(30, .8);
    Robot.kIntake.speedsuck(.5);
    Robot.kDrivetrain.justdrive(1);
    while(Robot.kElevator.Edoneyet != true || Robot.kElevator.Idoneyet != true){
    Robot.kElevator.compoundClimb(40, .3, 71, .3);
    }
   //Robot.kIntake.timedSuck(2, .5);
   Robot.kIntake.speedsuck(0);
  // Robot.kDrivetrain.justdrive(.5);
   
   //front.set(Value.kReverse);
  }
  

}
