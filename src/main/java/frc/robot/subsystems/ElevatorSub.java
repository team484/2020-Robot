/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.JoystickElevator;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;

public class ElevatorSub extends SubsystemBase {
  /**
   * Creates a new Elevator.
   */
  public ElevatorSub() {
    closeClutch();
    setDefaultCommand(new JoystickElevator(this));
    RobotIO.rightElevatorMotor.follow(RobotIO.leftElevatorMotor, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public static void openClutch(){
    RobotIO.clutchServo.setAngle(RobotSettings.CLUTCH_ENGAGE_ANGLE);
  }

  public static void closeClutch(){
    RobotIO.clutchServo.setAngle(RobotSettings.CLUTCH_DISENGAGE_ANGLE);
  }
  
  public static void set(double speed) {
    double height = getHeight();
    if (height > RobotSettings.ELEVATOR_MAX_HEIGHT && speed > 0) {
      RobotIO.leftElevatorMotor.set(0);
      return;
    }
    if (height <= 0 && speed < 0) {
      RobotIO.leftElevatorMotor.set(0);
      return;
    }
    RobotIO.leftElevatorMotor.set(speed);
  }

  public static double getrightHeight(){
    return RobotIO.rightElevatorMotor.getEncoder().getPosition();
  }
  
  public static double getleftHeight(){
  return RobotIO.leftElevatorMotor.getEncoder().getPosition();
  }

  public static double getHeight(){
  return Math.max(getleftHeight(), getrightHeight());
  }
  
}
