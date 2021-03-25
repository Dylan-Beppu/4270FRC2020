package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
//import 
//import com.revrobotics.CANEncoder;
//import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units; // units class converts imperial to si units 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;


import frc.robot.Constants.DriveConstants;


@SuppressWarnings("import unused")

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends SubsystemBase {
  private double X;
  private double Y;
  private double R;
  private boolean ToBallState = false;

  private static final double kGearRatio = 22; // gear ratio
  private static final double kWheelRadiusInches = 2.0;

   //Right drive
  private final WPI_TalonFX rightMaster = RobotMap.rightdrive1;
  private final WPI_TalonFX rightSub = RobotMap.rightdrive2;

 
  //Left drive
  private final WPI_TalonFX leftMaster = RobotMap.leftdrive1;
  private final WPI_TalonFX leftSub = RobotMap.leftdrive1;

  //private double ticksPerMeater = 213649;
  private double deadzoneleft = 0.10;
  private double deadzoneright = 0.10;
  
  private AHRS Gyro = RobotMap.gyro;

  //DifferentialDriveKinematics kinematics2 = new DifferentialDriveKinematics(Units.inchesToMeters(28)); //inches between distance between wheels
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24));
  
  //DifferentialDriveOdometry odometry2 = new DifferentialDriveOdometry(kinematics2, getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.3, 1.96, 0.06); 

  PIDController leftPIDController = new PIDController(1, 0, 0);
  PIDController rightPIDController = new PIDController(1, 0, 0);
  
  Pose2d pose = new Pose2d();

  public Drivetrain() {
    leftSub.follow(leftMaster);
    rightSub.follow(rightMaster);
    rightMaster.getSelectedSensorPosition(1);
    //leftMaster.setInverted(true);
    //rightMaster.setInverted(true);

    //Gryo.reset(); //scrued up spelling again LOL
    Gyro.reset();
  }

  public void lowGear(){
    Robot.kShifter.isfast = false;
  }

  public void highGear(){
    Robot.kShifter.isfast = true;
  }


  public void tank(){
    if(Robot.kShifter.isfast == true){
      if(Math.abs(Robot.m_oi.Driver.getRawAxis(1)*-1) > deadzoneleft){
        leftMaster.set(ControlMode.PercentOutput, Robot.m_oi.Driver.getRawAxis(1)*0.4);//swich val 1
         leftSub.set(ControlMode.PercentOutput, Robot.m_oi.Driver.getRawAxis(1)*0.4);//swich val 1

      }
      else{
        leftMaster.set(ControlMode.PercentOutput, 0);
        leftSub.set(ControlMode.PercentOutput, 0);
      }
      if(Math.abs(Robot.m_oi.Driver.getRawAxis(5)*-1) > deadzoneright){
        rightMaster.set(ControlMode.PercentOutput, Robot.m_oi.Driver.getRawAxis(5)*0.4); //swich val 3
        rightSub.set(ControlMode.PercentOutput, Robot.m_oi.Driver.getRawAxis(5)*0.4);//swich val 3
      }
      else{
        rightMaster.set(ControlMode.PercentOutput,0);
        rightSub.set(ControlMode.PercentOutput, 0);
      }
    }
    //when slow
    else{
      if(Math.abs(Robot.m_oi.Driver.getRawAxis(1)*-1) > deadzoneleft){
        leftMaster.set(ControlMode.PercentOutput, Robot.m_oi.Driver.getRawAxis(1));//swich val 1
        leftSub.set(ControlMode.PercentOutput, Robot.m_oi.Driver.getRawAxis(1));//swich val 1
      }
      else{
        leftMaster.set(ControlMode.PercentOutput, 0);
        leftSub.set(ControlMode.PercentOutput, 0);
      }
      if(Math.abs(Robot.m_oi.Driver.getRawAxis(5)*-1) > deadzoneright){
        rightMaster.set(ControlMode.PercentOutput, Robot.m_oi.Driver.getRawAxis(5));  //swich val 3
        rightSub.set(ControlMode.PercentOutput, Robot.m_oi.Driver.getRawAxis(5));      //swich val 3
      }
      else{
        rightMaster.set(ControlMode.PercentOutput,0);
        rightSub.set(ControlMode.PercentOutput, 0);
      } 
    }
  }


  public void autoBSpeed(double dirspd){
    RobotMap.rightdrive1.set(-dirspd);
    RobotMap.rightdrive2.set(-dirspd);
    RobotMap.leftdrive1.set(dirspd);
    RobotMap.leftdrive2.set(dirspd);
  }
  
  public void followSet(boolean Stateset){
    ToBallState = Stateset;
  }

  public void Camfollow(){
    //check distance not cam is rotated 90 deg
    if (ToBallState == true){
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("SmartDashboard");
    X = table.getEntry("X").getValue().getDouble();
    Y = table.getEntry("Y").getValue().getDouble();
    R = table.getEntry("R").getValue().getDouble();
      if (X > 130 && X < 255 && R < 20){
        if (Y > 90 && Y < 110){
          setOutputVolts(-3, -3);
        }else if(Y <= 90){
          setOutputVolts(-2, -3);
        }else if(Y >= 110){
          setOutputVolts(-3, -2);
        }
      }else{
        ToBallState = false;
        setOutputVolts(0, 0);
      }
      //ToBallState = false;
      
    }

  }
  public void setOutputVolts(double Ldrive,double Rdrive){
    RobotMap.rightdrive1.set(-Rdrive/12);
    RobotMap.rightdrive2.set(-Rdrive/12);
    RobotMap.leftdrive1.set(Ldrive/12);
    RobotMap.leftdrive2.set(Ldrive/12);
  }

  //Wapoint starts here ------------------------------------------------------------------------------------------

}
