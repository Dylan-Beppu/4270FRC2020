package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Robot;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static java.util.Map.entry;

public class RobotContainer {
  //private final RobotMap kRobotMap = new RobotMap();
  private final Drivetrain drive = new Drivetrain();
  //private final Turret kTurret = new Turret();
  //private final Shooter kShooter = new Shooter(kTurret);
  //boolean kAutoMode ;
  //private final
  private enum CommandSelector {
    Red1, Red2, Blue1, Blue2
  }
  
  // An example selector method for the selectcommand.  Returns the selector that will select
  // which command to run.  Can base this choice on logical conditions evaluated at runtime.
  private CommandSelector select() {
    return CommandSelector.Red1;
    //return CommandSelector.Red2;
  }

  // An example selectcommand.  Will select from the three commands based on the value returned
  // by the selector method at runtime.  Note that selectcommand works on Object(), so the
  // selector does not have to be an enum; it could be any desired type (string, integer,
  // boolean, double...)

    public RobotContainer(){
      
    
    //Transform2d transform = new Pose2d(4,4, Rotation2d.fromDegrees(50)).minus(trajectory.getInitialPose());
    //Trajectory newTrajectory = trajectory.transformBy(transform);
    }

    

    public Command getAutonomousCommand() {
    
      TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2.0), Units.feetToMeters(2.0));
      config.setKinematics(drive.getKinematics());
      
      //pose2d is seting the waypoints, try make diffrent trjectores rather than just one, may need a sepreat file for each one
      //or change it to pathweaver paths
    String trajectoryJSON = "paths/3ball1.json";
    //Trajectory trajectory = null;
    Trajectory trajectory = new Trajectory(new ArrayList<Trajectory.State>());
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      SmartDashboard.putBoolean("haspathbeenfound", true);
    } catch (IOException ex){
      DriverStation.reportError("Unable to open trajectory:" + trajectoryJSON, ex.getStackTrace());
      SmartDashboard.putBoolean("haspathbeenfound", false);
    }
    
      
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
