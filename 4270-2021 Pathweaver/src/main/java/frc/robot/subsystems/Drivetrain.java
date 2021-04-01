package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
//import 
//import com.revrobotics.CANEncoder;
//import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
//import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units; // units class converts imperial to si units 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.Filesystem;
//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
//import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
//import edu.wpi.first.wpilibj.geometry.Pose2d;
//import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
//import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
//import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
//import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

@SuppressWarnings("import unused")

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Drivetrain extends SubsystemBase {
  private double X;
  private double Y;
  private double R;
  private boolean ToBallState = false;

  // private static final double kGearRatio = 22; // gear ratio
  // private static final double kWheelRadiusInches = 2.0;

  // Right drive
  private final WPI_TalonFX rightMaster = RobotMap.rightdrive1;
  private final WPI_TalonFX rightSub = RobotMap.rightdrive2;

  // Left drive
  private final WPI_TalonFX leftMaster = RobotMap.leftdrive1;
  private final WPI_TalonFX leftSub = RobotMap.leftdrive1;

  // private double ticksPerMeater = 213649;
  private double deadzoneleft = 0.10;
  private double deadzoneright = 0.10;

  private AHRS m_gyro = RobotMap.gyro;

  PIDController leftPIDController = new PIDController(1, 0, 0);
  PIDController rightPIDController = new PIDController(1, 0, 0);

  Pose2d pose = new Pose2d();
  private double leftval = 0;
  private double rightval = 0;

  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(leftMaster, leftSub);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(rightMaster, rightSub);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  public Drivetrain() {
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    m_gyro.reset();
  }

  public void lowGear() {
    Robot.kShifter.isfast = false;
  }

  public void highGear() {
    Robot.kShifter.isfast = true;
  }

  public void tank() {
    if (Robot.kShifter.isfast == true) {
      if (Math.abs(Robot.m_oi.Driver.getRawAxis(1) * -1) > deadzoneleft) {
        leftval =  Robot.m_oi.Driver.getRawAxis(1) * 0.4;        
      } else {
        leftval = 0;        
      }
      if (Math.abs(Robot.m_oi.Driver.getRawAxis(5) * -1) > deadzoneright) {
        rightval =  Robot.m_oi.Driver.getRawAxis(5) * 0.4;  
      } else {
        rightval = 0;
      }
    }
    // when slow
    else {
      if (Math.abs(Robot.m_oi.Driver.getRawAxis(1) * -1) > deadzoneleft) {
        leftval = Robot.m_oi.Driver.getRawAxis(1);
      } else {
        leftval = 0;  
      }
      if (Math.abs(Robot.m_oi.Driver.getRawAxis(5) * -1) > deadzoneright) {
        rightval =  Robot.m_oi.Driver.getRawAxis(5);  
      } else {
        rightval = 0;
      }
    }
    m_drive.tankDrive(leftval, rightval);
  }

  public void autoBSpeed(double dirspd) {
    this.m_rightMotors.set(-dirspd);
    this.m_leftMotors.set(dirspd);
  }

  public void followSet(boolean Stateset) {
    ToBallState = Stateset;
  }

  public void Camfollow() {
    // check distance not cam is rotated 90 deg
    if (ToBallState == true) {
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      NetworkTable table = inst.getTable("SmartDashboard");
      X = table.getEntry("X").getValue().getDouble();
      Y = table.getEntry("Y").getValue().getDouble();
      R = table.getEntry("R").getValue().getDouble();
      if (X > 130 && X < 255 && R < 20) {
        if (Y > 90 && Y < 110) {
          setOutputVolts(-3, -3);
        } else if (Y <= 90) {
          setOutputVolts(-2, -3);
        } else if (Y >= 110) {
          setOutputVolts(-3, -2);
        }
      } else {
        ToBallState = false;
        setOutputVolts(0, 0);
      }
      // ToBallState = false;

    }

  }

  public void setOutputVolts(double Ldrive, double Rdrive) {
    RobotMap.rightdrive1.set(-Rdrive / 12);
    RobotMap.rightdrive2.set(-Rdrive / 12);
    RobotMap.leftdrive1.set(Ldrive / 12);
    RobotMap.leftdrive2.set(Ldrive / 12);
  }

  // Wapoint starts here
  // ------------------------------------------------------------------------------------------

  private double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = sensorCounts / DriveConstants.kEncoderCPR;
    double wheelRotations = motorRotations / DriveConstants.kGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelDiameterInc/2));
    return positionMeters;
  }

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorPosition()),
        nativeUnitsToDistanceMeters(rightMaster.getSelectedSensorPosition()));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(),
    // m_rightEncoder.getRate());
    return new DifferentialDriveWheelSpeeds(nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorVelocity()),
        nativeUnitsToDistanceMeters(rightMaster.getSelectedSensorVelocity()));

  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(-leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
    // m_leftEncoder.reset();
    // m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorPosition())
        + nativeUnitsToDistanceMeters(rightMaster.getSelectedSensorPosition())) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    m_gyro.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  //Follow the path baced on pathweavjson
  public Command FollowPath(String trajectoryJSON) {
    // String trajectoryJSON = "Paths/output/Unnamed.wpilib.json";

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/"+trajectoryJSON);

      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

      RamseteCommand ramseteCommand = new RamseteCommand(trajectory, this::getPose,
          new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
              DriveConstants.kaVoltSecondsSquaredPerMeter),
          DriveConstants.kDriveKinematics, this::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0),
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          this::tankDriveVolts, this);

      // Reset odometry to the starting pose of the trajectory.
      this.resetOdometry(trajectory.getInitialPose());

      // Run path following command, then stop at the end.
      return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
    } catch (IOException e) {

      System.out.println("Unable to open trajectory: " + trajectoryJSON);
      return null;
    }
  }
}
