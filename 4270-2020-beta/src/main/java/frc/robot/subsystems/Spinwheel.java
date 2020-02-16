package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.SparkMax;

import frc.robot.Robot;
import frc.robot.RobotMap;


public class Spinwheel extends SubsystemBase {

  private final CANSparkMax R2D2 = RobotMap.spin1;


  private final ColorSensorV3 colorSensor = RobotMap.colorSensor;
  private final ColorMatch colorMatcher = RobotMap.colorMatcher;

  private final Color kBlueTarget = RobotMap.kBlueTarget;
  private final Color kGreenTarget = RobotMap.kGreenTarget;
  private final Color kRedTarget = RobotMap.kRedTarget;
  private final Color kYellowTarget = RobotMap.kYellowTarget;

  Color detectedColor = colorSensor.getColor();
  String colorString;
  ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
  double counter1;
  double Tempcolor;

  public void ColorSpin(Double Rotations){
    Tempcolor = 0;
    if(colorString == "Blue" && Tempcolor == 0){
      Tempcolor = 1;
    }
    else if(colorString == "Blue" && Tempcolor == 0){
      Tempcolor = 2;
    }
    else if(colorString == "Blue" && Tempcolor == 0){
      Tempcolor = 3;
    }
    else if(colorString == "Blue" && Tempcolor == 0){
      Tempcolor = 4;
    }
    if(Tempcolor == 1){
      counter1 = +1;
    }
    else if(Tempcolor == 2){
      counter1 = +1;
    }
    else if(Tempcolor == 3){
      counter1 = +1;
    }
    else if(Tempcolor == 4){
      counter1 = +1;
    }
    if(counter1 < Rotations*2){
      R2D2.set(0.5);
    }
    else{
      R2D2.set(0);
      counter1 = 0;
      Tempcolor = 0;
    }


  }

  public void RotateToColor(Color Colorval){
    if(match.color == Colorval){
      R2D2.set(0);
    }
    else{
      R2D2.set(0.5);
    }
  }
  public void RotateColor(){
    if(Robot.m_oi.BtnPanle.getRawButton(1) == true){
      RotateToColor(kBlueTarget);
    }
    else if(Robot.m_oi.BtnPanle.getRawButton(1) == true){
      RotateToColor(kRedTarget);
    }
    else if(Robot.m_oi.BtnPanle.getRawButton(1) == true){
      RotateToColor(kGreenTarget);
    }
    else if(Robot.m_oi.BtnPanle.getRawButton(1) == true){
      RotateToColor(kYellowTarget);
    }
    else{
      R2D2.set(0);
    }
  }
  public void colorDectect() {
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
