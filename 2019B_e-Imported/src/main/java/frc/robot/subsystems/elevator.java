/*----------------------------------------------------------------------------*/
/* Copyright (c) 20.0118 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.TargetElevator;

/**
 * Add your docs here.
 */
public class elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final CANSparkMax eley = RobotMap.Melevator;
  private final CANSparkMax feley = RobotMap.Felevator;

  private final CANEncoder encoder = RobotMap.Melevator.getEncoder();

  private final DoubleSolenoid arm = RobotMap.arm;
 // private final DoubleSolenoid Intakearm = RobotMap.SolenoidIntake;
  private final CANSparkMax intakewrist = RobotMap.wristmotor;
  private final CANSparkMax fintakewrist = RobotMap.wristmotor2;
  private final CANEncoder wristencoder = RobotMap.wristmotor.getEncoder();



  public boolean doneyet;
  public boolean intUP;
  public boolean Edoneyet;
  public boolean Idoneyet;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TargetElevator());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void Up(double speed){
    eley.set(speed);
    feley.set(-speed);

  }

  public void Down(double speed){
    if(RobotMap.BottomLimit.get() != false){
    eley.set(-speed);
    feley.set(speed);
  }
  else if(RobotMap.BottomLimit.get() == false){
    eley.set(0.01);
    feley.set(-0.01);
    encoder.setPosition(0);
  }
  else{
    eley.set(0.01);
    feley.set(-0.01);
  }
  }

  public void complexElevate(double TargetPos, double maxspeed){
    Robot.kDrivetrain.tank(Robot.m_oi.stick);

    while (encoder.getPosition() < (TargetPos - 2) ){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);

      eley.set(maxspeed - (maxspeed * Math.pow(Math.E, (encoder.getPosition() - TargetPos))));
      feley.set(-(maxspeed - (maxspeed * Math.pow(Math.E, (encoder.getPosition() - TargetPos)))));
    }
    while (encoder.getPosition() > (TargetPos + 2) && RobotMap.BottomLimit.get() != false){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);

      eley.set((maxspeed * Math.pow(Math.E, (TargetPos - encoder.getPosition()))) - maxspeed);
      feley.set(-((maxspeed * Math.pow(Math.E, (TargetPos - encoder.getPosition()))) - maxspeed));
    }
    
  if(encoder.getPosition() > (TargetPos + 2) || encoder.getPosition() < (TargetPos - 2)){
    doneyet = false;
  }
  else{
    doneyet = true;
  }
    
    eley.set(0.01);
    feley.set(-0.01);

    if(RobotMap.BottomLimit.get() == false){
      encoder.setPosition(0);
    }

  }

  public void calmerComplex(double TargetPos, double maxspeed){
    Robot.kDrivetrain.tank(Robot.m_oi.stick);

    while (encoder.getPosition() < (TargetPos - 2)){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);
      eley.set(maxspeed - (maxspeed * Math.pow(Math.E, (4*(encoder.getPosition() - TargetPos)/5) )));
      feley.set(-(maxspeed - (maxspeed * Math.pow(Math.E, (4*(encoder.getPosition() - TargetPos)/5) ))));
    }
    while (encoder.getPosition() > (TargetPos + 2) && RobotMap.BottomLimit.get() != false){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);

      eley.set((maxspeed * Math.pow(Math.E, ((TargetPos - encoder.getPosition())/3) )) - maxspeed);
      feley.set(-((maxspeed * Math.pow(Math.E, ((TargetPos - encoder.getPosition())/3) )) - maxspeed));
    }
    
  if(encoder.getPosition() > (TargetPos + 2) || encoder.getPosition() < (TargetPos - 2)){
    doneyet = false;
  }
  else{
    doneyet = true;
  }
    eley.set(0.01);
    feley.set(-0.01);

    if(RobotMap.BottomLimit.get() == false){
      encoder.setPosition(0);
    }
  }

//
// fourbar portion
//

public void armup(){
  arm.set(Value.kForward);
}
public void armdown(){
  arm.set(Value.kReverse);
}

/*
  if(encoder.getPosition() > (TargetPos + 2) || encoder.getPosition() < (TargetPos - 2)){
    doneyet = false;
  }
  else{
    doneyet = true;
  }

*/
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
    doneyet = false;
  }
  else{
    doneyet = true;
  }
    
    intakewrist.set(0);
    fintakewrist.set(0);
  //  intUP = false;
   // feley.set(-0.01);
  if(-wristencoder.getPosition() > 25){
    intUP = false;
  }
  else{
    intUP = true;
  }

   // intUP = true;
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
    
   // feley.set(-0.01);
    }
    if(-wristencoder.getPosition() > 25){
      intUP = false;
    }
    else{
      intUP = true;
    }
  }
  public void manintakeup(){
    intakewrist.set( .4);
    fintakewrist.set(-.4);
    if(-wristencoder.getPosition() > 25){
      intUP = false;
    }
    else{
      intUP = true;
    }
    //Intakearm.set(Value.kReverse);
  //  intUP = true;
  }

  public void manintakedown(){
    intakewrist.set( -.1);
    fintakewrist.set(.1);
    if(-wristencoder.getPosition() > 25){
      intUP = false;
    }
    else{
      intUP = true;
    }
   // Intakearm.set(Value.kForward);
   // intUP = false;
  }
  public void manintakeoff(){
    intakewrist.set( 0);
    fintakewrist.set(0);
    if(-wristencoder.getPosition() > 25){
      intUP = false;
    }
    else{
      intUP = true;
    }
   // Intakearm.set(Value.kForward);
  }










  public void compoundClimb(double TargetPosE, double maxspeedE, double TargetPosI, double maxspeedI){
    Robot.kDrivetrain.tank(Robot.m_oi.stick);
/////////////////////////////////


    while (encoder.getPosition() < (TargetPosE - 2) && Math.abs(wristencoder.getPosition()) > (TargetPosI + 2) ){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);

      eley.set(maxspeedE - (maxspeedE * Math.pow(Math.E, (encoder.getPosition() - TargetPosE))));
      feley.set(-(maxspeedE - (maxspeedE * Math.pow(Math.E, (encoder.getPosition() - TargetPosE)))));
      fintakewrist.set((maxspeedI * Math.pow(Math.E, (TargetPosI - Math.abs(wristencoder.getPosition())))) - maxspeedI);
      intakewrist.set(-((maxspeedI * Math.pow(Math.E, (TargetPosI - Math.abs(wristencoder.getPosition())))) - maxspeedI));
    
    }
    while (encoder.getPosition() > (TargetPosE + 2) && RobotMap.BottomLimit.get() != false && Math.abs(wristencoder.getPosition()) > (TargetPosI + 2)){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);

      eley.set((maxspeedE * Math.pow(Math.E, (TargetPosE - encoder.getPosition()))) - maxspeedE);
      feley.set(-((maxspeedE * Math.pow(Math.E, (TargetPosE - encoder.getPosition()))) - maxspeedE));
      fintakewrist.set((maxspeedI * Math.pow(Math.E, (TargetPosI - Math.abs(wristencoder.getPosition())))) - maxspeedI);
      intakewrist.set(-((maxspeedI * Math.pow(Math.E, (TargetPosI - Math.abs(wristencoder.getPosition())))) - maxspeedI));
    
    }
    // Above is when intake is lower than needed
    /////////////////////
    // Below is when intake is higher than needed
    while (encoder.getPosition() < (TargetPosE - 2) && Math.abs(wristencoder.getPosition()) < (TargetPosI - 2) ){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);

      eley.set(maxspeedE - (maxspeedE * Math.pow(Math.E, (encoder.getPosition() - TargetPosE))));
      feley.set(-(maxspeedE - (maxspeedE * Math.pow(Math.E, (encoder.getPosition() - TargetPosE)))));
      fintakewrist.set(maxspeedI - (maxspeedI * Math.pow(Math.E, (Math.abs(wristencoder.getPosition()) - TargetPosI))));
      intakewrist.set(-(maxspeedI - (maxspeedI * Math.pow(Math.E, (Math.abs(wristencoder.getPosition()) - TargetPosI)))));
    
    }
    while (encoder.getPosition() > (TargetPosE + 2) && RobotMap.BottomLimit.get() != false && Math.abs(wristencoder.getPosition()) < (TargetPosI - 2)){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);

      eley.set((maxspeedE * Math.pow(Math.E, (TargetPosE - encoder.getPosition()))) - maxspeedE);
      feley.set(-((maxspeedE * Math.pow(Math.E, (TargetPosE - encoder.getPosition()))) - maxspeedE));
      fintakewrist.set(maxspeedI - (maxspeedI * Math.pow(Math.E, (Math.abs(wristencoder.getPosition()) - TargetPosI))));
      intakewrist.set(-(maxspeedI - (maxspeedI * Math.pow(Math.E, (Math.abs(wristencoder.getPosition()) - TargetPosI)))));
    
    }
    //////////////////////
    // Below is when elevator is ay-okay
    while (encoder.getPosition() < (TargetPosE + 2) && encoder.getPosition() > (TargetPosE - 2) && Math.abs(wristencoder.getPosition()) < (TargetPosI - 2)){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);

      eley.set(-0.02);
      feley.set(0.02);
      fintakewrist.set(maxspeedI - (maxspeedI * Math.pow(Math.E, (Math.abs(wristencoder.getPosition()) - TargetPosI))));
      intakewrist.set(-(maxspeedI - (maxspeedI * Math.pow(Math.E, (Math.abs(wristencoder.getPosition()) - TargetPosI)))));
    }
    while (encoder.getPosition() < (TargetPosE + 2) && encoder.getPosition() > (TargetPosE - 2) && Math.abs(wristencoder.getPosition()) > (TargetPosI + 2)){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);

      eley.set(0);
      feley.set(0);
      fintakewrist.set((maxspeedI * Math.pow(Math.E, (TargetPosI - Math.abs(wristencoder.getPosition())))) - maxspeedI);
      intakewrist.set(-((maxspeedI * Math.pow(Math.E, (TargetPosI - Math.abs(wristencoder.getPosition())))) - maxspeedI));
    }
    ////////////////
    // Below is when intake is ay-okay
    while (encoder.getPosition() < (TargetPosE - 2) && Math.abs(wristencoder.getPosition()) < (TargetPosI + 2) && Math.abs(wristencoder.getPosition()) > (TargetPosI - 2)){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);

      eley.set(maxspeedE - (maxspeedE * Math.pow(Math.E, (encoder.getPosition() - TargetPosE))));
      feley.set(-(maxspeedE - (maxspeedE * Math.pow(Math.E, (encoder.getPosition() - TargetPosE)))));
      intakewrist.set(0);
    fintakewrist.set(0);
    }
    while (encoder.getPosition() > (TargetPosE + 2) && RobotMap.BottomLimit.get() != false && Math.abs(wristencoder.getPosition()) < (TargetPosI + 2) && Math.abs(wristencoder.getPosition()) > (TargetPosI - 2)){
      Robot.kDrivetrain.tank(Robot.m_oi.stick);

      eley.set((maxspeedE * Math.pow(Math.E, (TargetPosE - encoder.getPosition()))) - maxspeedE);
      feley.set(-((maxspeedE * Math.pow(Math.E, (TargetPosE - encoder.getPosition()))) - maxspeedE));
      intakewrist.set(0);
    fintakewrist.set(0);
    }
   
    

  //////////////////////////////  
  if(encoder.getPosition() > (TargetPosE + 2) || encoder.getPosition() < (TargetPosE - 2)){
    Edoneyet = false;
  }
  else{
    Edoneyet = true;
  }
  if(Math.abs(wristencoder.getPosition()) > (TargetPosI + 2) || Math.abs(wristencoder.getPosition()) < (TargetPosI - 2)){
    Idoneyet = false;
  }
  else{
    Idoneyet = true;
  }
    
    eley.set(-0.02);
    feley.set(0.02);
    intakewrist.set(0);
    fintakewrist.set(0);

    if(RobotMap.BottomLimit.get() == false){
      encoder.setPosition(0);
    }

  }
}
