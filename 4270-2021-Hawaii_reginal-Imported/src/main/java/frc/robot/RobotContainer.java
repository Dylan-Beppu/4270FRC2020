package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
//import edu.wpi.first.wpilibj.geometry.Pose2d;
//import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.geometry.Transform2d;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
//import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
//import java.util.Arrays;
//import java.util.Map;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
//import frc.robot.commands.*;
//import static java.util.Map.entry;
import frc.robot.subsystems.Drivetrain;

//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  //private final RobotMap kRobotMap = new RobotMap();
  private final Drivetrain drive = new Drivetrain();
  //private String trajectoryJSON = Robot.trajectoryJSON;
  //private final Turret kTurret = new Turret();
  //private final Shooter kShooter = new Shooter(kTurret);
  //boolean kAutoMode ;
  //private final

  


  private enum CommandSelector {
    Red1, Red2, Blue1, Blue2
  }
  
  // An example selector method for the selectcommand.  Returns the selector that will select
  // which command to run.  Can base this choice on logical conditions evaluated at runtime.
  public CommandSelector select() {
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


    public Command getSelectedAuto(){
      //Robot.autoChooser.getSelected();
      return Robot.autoChooser.getSelected();
    }
    

   
    public void hi(){
      drive.tank();
      // kShooter.schedule();
      ///kShooter.execute();
      //drive.shift();
    }
    
  }
