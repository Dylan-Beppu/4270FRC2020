/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.*;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units; // units class converts imperial to si units
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import java.math.*;
import com.kauailabs.navx.frc.AHRS;
//import 


/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends SubsystemBase {

  private static final double kGearRatio = 7.29; // gear ratio
  private static final double kWheelRadiusInches = 3.0;

   //Right drive
  private final TalonSRX rightMaster = RobotMap.rightdrive1;
  private final TalonSRX rightSub = RobotMap.rightdrive2;

 
  //Left drive
  private final TalonSRX leftMaster = RobotMap.leftdrive1;
  private final TalonSRX leftSub = RobotMap.leftdrive1;

  private double deadzoneleft = 0.1;
  private double deadzoneright = 0.1;
  
  private AHRS Gyro = RobotMap.gyro;

  //DifferentialDriveKinematics kinematics2 = new DifferentialDriveKinematics(Units.inchesToMeters(28)); //inches between distance between wheels
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
  //DifferentialDriveOdometry odometry2 = new DifferentialDriveOdometry(kinematics2, getHeading());
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.3, 1.96, 0.06);

  PIDController leftPIDController = new PIDController(2.95, 0, 0);
  PIDController rightPIDController = new PIDController(2.95, 0, 0);

  Pose2d pose = new Pose2d();

  public Drivetrain() {
    leftSub.follow(leftMaster);
    rightSub.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    //Gryo.reset();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-Gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftMaster.getActiveTrajectoryVelocity(0) / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60, 
        rightMaster.getActiveTrajectoryVelocity(0) / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60
    );
  }

  public double getLvelocity(){
    return leftMaster.getActiveTrajectoryVelocity(0) / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60;
  }

  public double getRvelocity(){
    return rightMaster.getActiveTrajectoryVelocity(0) / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public void setOutputVolts(double leftVolts, double rightVolts) {
    leftMaster.configVoltageCompSaturation(leftVolts / 12);
    //leftMaster.set(leftVolts / 12);
    rightMaster.configVoltageCompSaturation(rightVolts / 12);
    //rightMaster.set(rightVolts / 12);
  }

  public void reset() {
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), getLvelocity(), getRvelocity());
  }


  
}
