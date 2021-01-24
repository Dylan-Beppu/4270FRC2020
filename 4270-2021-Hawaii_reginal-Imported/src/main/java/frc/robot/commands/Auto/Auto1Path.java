package frc.robot.commands.Auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

public class Auto1Path extends CommandBase {
  private final Drivetrain drive;
  
  public Auto1Path(Drivetrain subsystem){
    drive = subsystem;
    addRequirements(drive);
  }
  
  @Override
  public void execute(){
    String trajectoryJson = "./paths/streightTest.wpilib.json";
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson);
    
    try {
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      RamseteCommand command = new RamseteCommand(
      trajectory,
      drive::getPose,
      new RamseteController(2.0, 0.7),
      drive.getFeedforward(),
      drive.getKinematics(),
      drive::getSpeeds,
      drive.getLeftPIDController(),
      drive.getRightPIDController(),
      drive::setOutputVolts,
      drive
      );

      command.execute();
    } catch (IOException e) {
      System.out.println("Unable to load trajectory: " + trajectoryJson);
    }
  }
  
  @Override
  public void end(boolean interrupted){
    
  }

  @Override 
  public boolean isFinished(){
    return false;
  }
}