package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
/**
 * Add your docs here.
 */
public class RobotContainer {
    //private final RobotMap kRobotMap = new RobotMap();
    private final Drivetrain drive = new Drivetrain();
    //private final Turret kTurret = new Turret();
    //private final Shooter kShooter = new Shooter(kTurret);
    
    //private final

    public RobotContainer(){
      /*String trajectoryJSON = "paths/YOURPATH.wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex){
      DriverStation.reportError("Unable to open trajectory:" + trajectoryJSON, ex.getStackTrace());
    }
    Transform2d transform = new Pose2d(4,4, Rotation2d.fromDegrees(50)).minus(trajectory.getInitialPose());
    Trajectory newTrajectory = trajectory.transformBy(transform);*/
    }

    

    public Command getAutonomousCommand() {
      TrajectoryConfig config = new TrajectoryConfig(
          Units.feetToMeters(2.0), Units.feetToMeters(2.0));
      config.setKinematics(drive.getKinematics());
      
      //pose2d is seting the waypoints, try make diffrent trjectores rather than just one, may need a sepreat file for each one
      //or change it to pathweaver paths
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          Arrays.asList(new Pose2d(), new Pose2d(2, 0, new Rotation2d())
              /*new Pose2d(2, 0, Rotation2d.fromDegrees(0))*/),
          config
      );
      
      RamseteCommand command = new RamseteCommand(
          trajectory,
          drive::getPose,
          new RamseteController(2, .7),
          drive.getFeedforward(),
          drive.getKinematics(),
          drive::getSpeeds,
          drive.getLeftPIDController(),
          drive.getRightPIDController(),
          drive::setOutputVolts,
          drive
          
      );
  
      return command.andThen(() -> drive.setOutputVolts(0, 0));
    }
  
    public void reset() {
      drive.reset();
    }
    public void hi(){
      drive.tank();
      // kShooter.schedule();
      ///kShooter.execute();
      //drive.shift();
    }
  }
