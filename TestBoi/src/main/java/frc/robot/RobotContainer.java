/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.Drivetrain;

import java.util.Arrays;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * Add your docs here.
 */
public class RobotContainer {
    private Drivetrain drive = new Drivetrain();
  
    public Command getAutonomousCommand() {
      TrajectoryConfig config = new TrajectoryConfig(
          Units.feetToMeters(2.0), Units.feetToMeters(2.0));
      config.setKinematics(drive.getKinematics());
            
      //pose2d is seting the waypoints, try make diffrent trjectores rather than just one, may need a sepreat file for each one
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          Arrays.asList(new Pose2d(), new Pose2d(1, 0, new Rotation2d())
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
    }
  }
