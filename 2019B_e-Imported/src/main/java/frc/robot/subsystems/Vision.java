/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
//import edu.wpi.first.wpilibj.InterruptableSensorBase.WaitResult;
import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
//import frc.robot.commands.Visiontest;
//import frc.robot.driver.Limelight;
//import frc.robot.OI;
import frc.robot.commands.Visiontest;


/**
 * Add your docs here.
 */
public class vision extends Subsystem {
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ledmode;
  public double camhight;
  public double targethight;
  public boolean togglebtn;
  private double dts;
  public boolean vib;
  //NetworkTableEntry ty;

  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // Example: NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);

  // limelight values
  // tv	Whether the limelight has any valid targets (0 or 1)
  // tx	Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees) !!!
  // ty	Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
  // ta	Target Area (0% of image to 100% of image)
  // ts	Skew or rotation (-90 degrees to 0 degrees)
  // tl	The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.

  //talon can id 13
  //private double Xangle;


  private final CANSparkMax in = RobotMap.visionm;
  private final CANSparkMax intake = RobotMap.intake;
  private final CANSparkMax roller = RobotMap.side;

  //private final TalonSRX twist = RobotMap.VisionTurn;
  //private  Xangle = NetworkTableInstance.getDefault().getTable("table").getEntry("<ty>").getValue();
  //private frc.robot.driver.Limelight.;

  @Override
  public void initDefaultCommand() {
   setDefaultCommand(new Visiontest());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void LR(double speed){
    in.set(speed);
  }
  public void spinL(double speed){
    in.set(speed);
  }
  public void spinR(double speed){
    in.set(speed);
  }
  public void spinStop(){
    in.set(0);
  }
  
  public void blindMe(){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("limelight");
    ledmode = table.getEntry("ledMode");
    ledmode.setDouble(3);
  }
  public void unblindMe(){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("limelight");
    ledmode = table.getEntry("ledMode");
    ledmode.setDouble(1);
  }
  public void shootshoot(){
    if(Robot.m_oi.stick.getRawAxis(3) != 0){
      roller.set(1);
    }
    else{
      roller.set(0);
    }
  }
  public void camencreset(){
    if(Robot.m_oi.stick.getRawButton(9) == true){
      in.getEncoder().setPosition(0);
    }
    else{

    }
  }
  public void camPosReset(){
    in.setIdleMode(IdleMode.kBrake);
    in.getEncoder().getPosition();
    if(Robot.m_oi.stick.getRawButton(3) == true){
      if(in.getEncoder().getPosition() > 1){
        in.set(-0.5);
      }
      else if(1 > in.getEncoder().getPosition() && in.getEncoder().getPosition() > 0.1){
        in.set(-in.getEncoder().getPosition()/2);
      }
      else if(in.getEncoder().getPosition() < -1){
        in.set(0.5);
      }
      else if(in.getEncoder().getPosition() < -0.1 && in.getEncoder().getPosition() > -1){
        in.set(in.getEncoder().getPosition()/2);
      }
      else{
        in.set(0);
      }
    }
  }
  public void areweinrange(){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("limelight");
    camhight = 1.2573;
    targethight = 2.1082;
    ty = table.getEntry("ty");
    dts = (targethight-camhight)/ Math.tan(Math.toRadians(ty.getValue().getDouble()));
    if(dts < 5 && 4 < dts){
      Robot.m_oi.stick.setRumble(RumbleType.kLeftRumble, 1);
      Robot.m_oi.stick.setRumble(RumbleType.kRightRumble, 1);
    }
    else{
      Robot.m_oi.stick.setRumble(RumbleType.kLeftRumble, 0);
      Robot.m_oi.stick.setRumble(RumbleType.kRightRumble, 0);
    }
  }
  public void toggleon(){
    if(Robot.m_oi.stick.getRawButtonPressed(6)){
      if(togglebtn == true){
      togglebtn = false;
      }
      else{
        togglebtn = true;
      }
    }
  }
  
  public void track(){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("limelight");



    tx = table.getEntry("tx");

    if(togglebtn == true){
    
    intake.setOpenLoopRampRate(1);
    intake.set(1);
    intake.setIdleMode(IdleMode.kCoast);
    blindMe();
    
    }
    else{
      unblindMe();
      intake.set(0);
    }

    
    
    //ty = table.getEntry("ty");
    if(tx.getValue().getDouble() <= -1 && togglebtn == true){
      spinL(tx.getValue().getDouble()/-50);
      //in.set(-speed);
      //twist.set(ControlMode.PercentOutput, 0.5);
    }
    else if(tx.getValue().getDouble() >= 1 && togglebtn == true){
      //in.set(speed);
      spinR(tx.getValue().getDouble()/-50);
      
      //twist.set(ControlMode.PercentOutput, -0.5);
    }
    else{
      spinStop();
      
      //in.set(0);
      //twist.set(ControlMode.PercentOutput, 0);
    }
  }
}
