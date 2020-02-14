/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.elevator.ElevatorDoNothing;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;

public class ElevatorSub extends SubsystemBase {
  private static boolean wasClutchEverOpen = false;
  /**
   * Creates a new Elevator.
   */
  public ElevatorSub() {
    closeClutch();
    setDefaultCommand(new ElevatorDoNothing(this));
    RobotIO.leftElevatorMotor.follow(RobotIO.rightElevatorMotor, true);
  }

  @Override
  public void periodic() {
    if (DriverStation.getInstance().getMatchTime() < 1 && DriverStation.getInstance().isOperatorControl()) {
      openClutch();
    }
  }
  public static void openClutch(){
    wasClutchEverOpen = true;
    RobotIO.clutchServo.setAngle(RobotSettings.CLUTCH_ENGAGE_ANGLE);
  }

  public static void closeClutch(){
    RobotIO.clutchServo.setAngle(RobotSettings.CLUTCH_DISENGAGE_ANGLE);
  }
  
  public static void set(double speed) {
    double height = getHeight();
    if ((height > RobotSettings.ELEVATOR_MAX_HEIGHT || wasClutchEverOpen) && speed > 0) {
      RobotIO.rightElevatorMotor.set(0);
      return;
    }
    if (height <= 0 && speed < 0) {
      RobotIO.rightElevatorMotor.set(0);
      return;
    }
    RobotIO.rightElevatorMotor.set(speed);
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
