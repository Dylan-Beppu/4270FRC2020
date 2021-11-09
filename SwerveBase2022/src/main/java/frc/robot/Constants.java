// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.jni.CANSparkMaxJNI;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static AHRS gyro = new AHRS(Port.kUSB);

  public static final class OIConstants {
    //Note Opperator inputs are now in the constants file
    //TODO: add the xbox and switch configs (and maby ps5?)
    public static final Joystick Driver = new Joystick(0);
    
    public static final JoystickButton DriBtn1 = new JoystickButton(Driver, 1);
    public static final JoystickButton DriBtn2 = new JoystickButton(Driver, 2);
    public static final JoystickButton DriBtn3 = new JoystickButton(Driver, 3);
    public static final JoystickButton DriBtn4 = new JoystickButton(Driver, 4);
    public static final JoystickButton DriBtn5 = new JoystickButton(Driver, 5);
    public static final JoystickButton DriBtn6 = new JoystickButton(Driver, 6);
    public static final JoystickButton DriBtn7 = new JoystickButton(Driver, 7);
    public static final JoystickButton DriBtn8 = new JoystickButton(Driver, 8);
    public static final JoystickButton DriBtn9 = new JoystickButton(Driver, 9);
    public static final JoystickButton DriBtn10 = new JoystickButton(Driver, 10);
    
     
     
     //public Joystick BtnPanle = new Joystick(1);
  }

  public static final class DriveConstants {
    //public static final TalonSRX kRearLeftDriveMotor = new TalonSRX(2);
    public static final CANSparkMax kFrontLeftDriveMotor = new CANSparkMax(1, MotorType.kBrushless);
    public static final CANSparkMax kRearLeftDriveMotor = new CANSparkMax(2, MotorType.kBrushless);
    public static final CANSparkMax kFrontRightDriveMotor = new CANSparkMax(3, MotorType.kBrushless);
    public static final CANSparkMax kRearRightDriveMotor = new CANSparkMax(4, MotorType.kBrushless);
    public static final CANSparkMax kFrontLeftTurningMotor = new CANSparkMax(5, MotorType.kBrushless);
    public static final CANSparkMax kRearLeftTurningMotor = new CANSparkMax(6, MotorType.kBrushless);
    public static final CANSparkMax kFrontRightTurningMotor = new CANSparkMax(7, MotorType.kBrushless);
    public static final CANSparkMax kRearRightTurningMotor = new CANSparkMax(8, MotorType.kBrushless);


    //TODO: rename for canIDs actuly redo theis whole
    // public static final int kFrontLeftDriveMotorPort = 0;
    // public static final int kRearLeftDriveMotorPort = 2;
    // public static final int kFrontRightDriveMotorPort = 4;
    // public static final int kRearRightDriveMotorPort = 6;

    // public static final int kFrontLeftTurningMotorPort = 1;
    // public static final int kRearLeftTurningMotorPort = 3;
    // public static final int kFrontRightTurningMotorPort = 5;
    // public static final int kRearRightTurningMotorPort = 7;
    //TODO: figure out the encoder stuff and find out if the ones on the mk2 modules can substitute in
    //TODO: also see if the encoders can be swaped for the intergrated ones (so redundent other one???)
    // public static final int[] kFrontLeftTurningEncoderPorts = new int[] {0, 1};
    // public static final int[] kRearLeftTurningEncoderPorts = new int[] {2, 3};
    // public static final int[] kFrontRightTurningEncoderPorts = new int[] {4, 5};
    // public static final int[] kRearRightTurningEncoderPorts = new int[] {5, 6};

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = true;

    // public static final int[] kFrontLeftDriveEncoderPorts = new int[] {7, 8};
    // public static final int[] kRearLeftDriveEncoderPorts = new int[] {9, 10};
    // public static final int[] kFrontRightDriveEncoderPorts = new int[] {11, 12};
    // public static final int[] kRearRightDriveEncoderPorts = new int[] {13, 14};

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = true;

    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
