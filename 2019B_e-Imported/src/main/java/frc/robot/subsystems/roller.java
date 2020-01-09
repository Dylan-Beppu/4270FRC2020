/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.manualRoller;

/**
 * Add your docs here.
 */
public class roller extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final CANSparkMax intakewrist = RobotMap.wristmotor;
  private final CANSparkMax fintakewrist = RobotMap.wristmotor2;
  private final CANEncoder wristencoder = RobotMap.wristmotor.getEncoder();



  public boolean kdoneyet;
  public boolean kintUP;

  @Override
  public void initDefaultCommand() {
   // setDefaultCommand(new manualRoller());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void targetIntake(double TargetPos, double maxspeed){ /// UP IS NOW ALL THE WAY DOWN
    Robot.kDrivetrain.tank(Robot.m_oi.stick);

    while (Math.abs(wristencoder.getPosition()) < (TargetPos - 2) ){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);

      fintakewrist.set(maxspeed - (maxspeed * Math.pow(Math.E, (Math.abs(wristencoder.getPosition()) - TargetPos))));
      intakewrist.set(-(maxspeed - (maxspeed * Math.pow(Math.E, (Math.abs(wristencoder.getPosition()) - TargetPos)))));
    }
    while (Math.abs(wristencoder.getPosition()) > (TargetPos + 2)){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);

      fintakewrist.set((maxspeed * Math.pow(Math.E, (TargetPos - Math.abs(wristencoder.getPosition())))) - maxspeed);
      intakewrist.set(-((maxspeed * Math.pow(Math.E, (TargetPos - Math.abs(wristencoder.getPosition())))) - maxspeed));
    }
    
  if(Math.abs(wristencoder.getPosition()) > (TargetPos + 2) || Math.abs(wristencoder.getPosition()) < (TargetPos - 2)){
    kdoneyet = false;
  }
  else{
    kdoneyet = true;
  }
    
    intakewrist.set(0);
    fintakewrist.set(0);
    kintUP = false;
   // feley.set(-0.01);


   // kintUP = true;
  }

  public void intakeup(){
    Robot.kDrivetrain.tank(Robot.m_oi.stick);

    if (Math.abs(wristencoder.getPosition()) < (0 - 50) ){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);

      intakewrist.set( (.7 - (.7 * Math.pow(Math.E, ((Math.abs(wristencoder.getPosition()) - 0) / 1000) ))));
    //  feley.set(-(maxspeed - (maxspeed * Math.pow(Math.E, (encoder.getPosition() - TargetPos)))));
    }
    else if (Math.abs(wristencoder.getPosition()) > (0 + 50)){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);

      intakewrist.set( (1 * Math.pow(Math.E, ((0 - Math.abs(wristencoder.getPosition())) / 1000 )) - 1));
     // feley.set(-((maxspeed * Math.pow(Math.E, (TargetPos - encoder.getPosition()))) - maxspeed));
    }
    else{
      Robot.kDrivetrain.tank(Robot.m_oi.stick);
    intakewrist.set( -0.05);
    kintUP = true;
   // feley.set(-0.01);
    }
  }
  public void manintakeup(){
    intakewrist.set( .4);
    fintakewrist.set(-.7);
    //Intakearm.set(Value.kReverse);
  //  kintUP = true;
  }

  public void manintakedown(){
    intakewrist.set( -.5);
    fintakewrist.set(.5);
   // Intakearm.set(Value.kForward);
   // kintUP = false;
  }
  public void manintakeoff(){
    intakewrist.set( 0);
    fintakewrist.set(0);
   // Intakearm.set(Value.kForward);
  }
}
