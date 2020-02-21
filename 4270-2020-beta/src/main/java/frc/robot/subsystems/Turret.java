package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.jni.CANSparkMaxJNI;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.InterruptableSensorBase.WaitResult;
//import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.buttons.Button;
//import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
//import frc.robot.commands.Visiontest;
//import frc.robot.driver.Limelight;
//import frc.robot.OI;
import frc.robot.commands.Shooter;


/**
 * Add your docs here.
 */

public class Turret extends SubsystemBase {
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


  private final CANSparkMax FLyBoiR = RobotMap.FlyboiR;
  private final CANSparkMax Rotateboi = RobotMap.Rotateboi;
  private final CANSparkMax FLyBoiL = RobotMap.FlyboiL;

  private final CANSparkMax TopIndex = RobotMap.Topin;
  private final CANSparkMax Bindex = RobotMap.IndexBottom;
  double turretENCoffset;
  
  //private final TalonSRX twist = RobotMap.VisionTurn;
  //private  Xangle = NetworkTableInstance.getDefault().getTable("table").getEntry("<ty>").getValue();
  //private frc.robot.driver.Limelight.;

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
  public void LR(double speed){
    RobotMap.Rotateboi.set(speed);
  }
  public void spinL(double speed){
    RobotMap.Rotateboi.set(speed);
  }
  public void spinR(double speed){
    RobotMap.Rotateboi.set(speed);
  }
  public void spinStop(){
    RobotMap.Rotateboi.set(0);
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
    if(Robot.m_oi.BailysJob.getRawAxis(3) != 0){
      RobotMap.CenterIntake.set(0.5);
      Bindex.set(-0.7);
    }
    else{
      Bindex.set(0);
      RobotMap.CenterIntake.set(0);
    }
  }
  
  public void camencreset(){
    if(Robot.m_oi.BailysJob.getRawButton(9) == true){
      RobotMap.Rotateboi.getEncoder().setPosition(0);
    }
    else{

    }
  }
  //turet max allowed 90deg
  //turet absolute max 100-105 deg
  //21 rotations|| 0 rotation
  public void camPosReset(){
    Rotateboi.setIdleMode(IdleMode.kBrake);
    Rotateboi.getEncoder().getPosition();
    turretENCoffset = 1;
    if(Robot.m_oi.BailysJob.getRawButton(3) == true){
      if(Rotateboi.getEncoder().getPosition() > 1){
        RobotMap.Rotateboi.set(-0.5);
      }
      else if(1 > Rotateboi.getEncoder().getPosition() && Rotateboi.getEncoder().getPosition() > 0.1){
        RobotMap.Rotateboi.set(-Rotateboi.getEncoder().getPosition()/2);
      }
      else if(Rotateboi.getEncoder().getPosition() < -1){
        RobotMap.Rotateboi.set(0.5);
      }
      else if(Rotateboi.getEncoder().getPosition() < -0.1 && Rotateboi.getEncoder().getPosition() > -1){
        RobotMap.Rotateboi.set(Rotateboi.getEncoder().getPosition()/2);
      }
      else{
        RobotMap.Rotateboi.set(0);
      }
    }
  }

  public void areweinrange(){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("limelight");
    camhight = Units.inchesToMeters(35);
    targethight = 2.1082;
    ty = table.getEntry("ty");
    dts = (targethight-camhight)/ Math.tan(Math.toRadians(ty.getValue().getDouble()+15));
    
    SmartDashboard.putNumber("Distants To target in m", dts);
    if(dts < 5 && 4 < dts){
      //Robot.m_oi.BailysJob.setRumble(RumbleType.kLeftRumble, 1);
      //Robot.m_oi.BailysJob.setRumble(RumbleType.kRightRumble, 1);
    }
    else{
      //Robot.m_oi.BailysJob.setRumble(RumbleType.kLeftRumble, 0);
      //Robot.m_oi.BailysJob.setRumble(RumbleType.kRightRumble, 0);
    }
  }
  public void toggleon(){
    if(Robot.m_oi.BailysJob.getRawButtonPressed(6)){
      if(togglebtn == true){
      togglebtn = false;
      }
      else{
        togglebtn = true;
      }
    }
  }
  
    //turet max allowed 90deg
  //turet absolute max 100-105 deg
  //21 rotations|| 0 rotation
  public void track(){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("limelight");

    tx = table.getEntry("tx");

    if(togglebtn == true){
    
      RobotMap.FlyboiR.setOpenLoopRampRate(0.7);
      RobotMap.FlyboiR.set(0.62);
      RobotMap.FlyboiR.setIdleMode(IdleMode.kCoast);
      RobotMap.FlyboiL.setOpenLoopRampRate(0.7);
      RobotMap.FlyboiL.set(-0.62);
      RobotMap.FlyboiL.setIdleMode(IdleMode.kCoast);
      RobotMap.Topin.set(-1);
      RobotMap.Topin.setIdleMode(IdleMode.kCoast);
    blindMe();
    
    }
    else{
      unblindMe();
      FLyBoiL.set(0);
      FLyBoiR.set(0);
      RobotMap.Topin.set(0);

    }

    
    
    //ty = table.getEntry("ty");&& Rotateboi.getEncoder().getPosition() < 21 && Rotateboi.getEncoder().getPosition() > -21
    if(tx.getValue().getDouble() <= -0.5 && togglebtn == true ){
      //spinL(tx.getValue().getDouble()/-30);
      //Rotateboi.set(-0.5);
      //twist.set(ControlMode.PercentOutput, 0.5);&& Rotateboi.getEncoder().getPosition() < 21 && Rotateboi.getEncoder().getPosition() > -21
    }
    else if(tx.getValue().getDouble() >= 0.5 && togglebtn == true ){
      //Rotateboi.set(0.5);
      //spinR(tx.getValue().getDouble()/ 30);
      
      //twist.set(ControlMode.PercentOutput, -0.5);
    }
    else{
      spinStop();
      
      //Rotateboi.set(0);
      //twist.set(ControlMode.PercentOutput, 0);
    }
  }
}
