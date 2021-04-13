// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
//see here for help with data analisis https://www.chiefdelphi.com/t/frc-robot-characterization-track-width-n-a/392068/15
public final class Constants {
  public static final class DriveConstants {

    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    //remembert to tune all the below for the frc robot using the Robot Characterization Toolsuite 
    //start from commandline using "frc-characterization drive new" see here for help file:///C:/Users/Public/wpilib/2021/documentation/rtd/frc-docs-latest/index.html#document-docs/software/wpilib-tools/robot-characterization/index
    //also use the right units
    
    public static final double kTrackwidthMeters = 0.3208593820918048; //not the same as robot base with thats sed in pathweaver
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    //base setup
    public static final double kEncoderCPR = 2048;
    public static final double kWheelDiameterInc = 6; 

    //SimpleMotorFeedforward varibles
    public static final double ksVolts = 0.531;
    public static final double kvVoltSecondsPerMeter = 12.2;
    public static final double kaVoltSecondsSquaredPerMeter = 0.725;

    //grar ratio for high gear setting
    public static final double kGearRatio = 27.27;
    
    //sets up pid controlers for ramsete command
    public static final double kPDriveVel = 0.709; 
    public static final double kIDriveVel = 0.0; 
    public static final double kDDriveVel = 0.0; 
  }

  public static final class AutoConstants {
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}