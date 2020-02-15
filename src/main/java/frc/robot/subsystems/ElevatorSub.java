/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    RobotIO.leftElevatorMotor.restoreFactoryDefaults();
    RobotIO.rightElevatorMotor.restoreFactoryDefaults();
    RobotIO.leftElevatorMotor.setSmartCurrentLimit(RobotSettings.ELEVATOR_MAX_CURRENT);
    RobotIO.rightElevatorMotor.setSmartCurrentLimit(RobotSettings.ELEVATOR_MAX_CURRENT);
  }

  @Override
  public void periodic() {
    if (DriverStation.getInstance().getMatchTime() < 1 && DriverStation.getInstance().getMatchTime() > 0 && DriverStation.getInstance().isOperatorControl()) {
      openClutch();
    }
    SmartDashboard.putNumber("Match time", DriverStation.getInstance().getMatchTime());
    SmartDashboard.putNumber("elevator", getHeight());
  }
  public static void openClutch(){
    wasClutchEverOpen = true;
    RobotIO.clutchServo.setAngle(RobotSettings.CLUTCH_ENGAGE_ANGLE);
  }

  public static void closeClutch(){
    RobotIO.clutchServo.setAngle(RobotSettings.CLUTCH_DISENGAGE_ANGLE);
  }
  
  public static void set(double speed) {
    SmartDashboard.putBoolean("Was Clutch Open", wasClutchEverOpen);
    SmartDashboard.putNumber("ElevatorSpeed", speed);
    double height = getHeight();
    if ((height > RobotSettings.ELEVATOR_MAX_HEIGHT || wasClutchEverOpen) && speed > 0) {
      setSc(0);
      return;
    }
    if (height < 1 && speed < 0) {
      setSc(0);
      return;
    }
    setSc(speed);
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

  private static void setSc(double speed) {
    RobotIO.leftElevatorMotor.set(speed);
    RobotIO.rightElevatorMotor.set(-speed);
  }
  
}
