/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.intaker;

/**
 * Add your docs here.
 */
public class intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final CANSparkMax motor = RobotMap.intake;
  private final CANSparkMax roller = RobotMap.side;

  public double sucktime;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new intaker());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void suck(){
    roller.set(.7);
    motor.set(.7);
  }
  public void speedsuck(double speed){
    roller.set(speed);
    motor.set(speed);
  }

  public void eject(){
    motor.set(-1);
    roller.set(0);
  }

  public void chill(){
    motor.set(0);
    roller.set(0);
  }
  public void timedSuck(double duration, double speed){
    sucktime = Timer.getFPGATimestamp();
    double desiredTime = sucktime + duration;
    while(Timer.getFPGATimestamp() < desiredTime)
    {
      roller.set(speed);
      motor.set(speed);
    }
    roller.set(0);
    motor.set(0);
  }
}
