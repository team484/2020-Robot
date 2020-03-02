/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.elevator.ElevatorDoNothing;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;

public class ElevatorSub extends SubsystemBase {
  private static boolean wasClutchEverOpen = false;
  public static boolean elevatorError = false;
  private static CANEncoder leftEncoder;
  private static CANEncoder rightEncoder;

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
    leftEncoder = RobotIO.leftElevatorMotor.getEncoder();
    rightEncoder = RobotIO.rightElevatorMotor.getEncoder();
  }

  @Override
  public void periodic() {
    if (DriverStation.getInstance().getMatchTime() < 1 && DriverStation.getInstance().getMatchTime() > 0
        && DriverStation.getInstance().isOperatorControl()) {
      openClutch();
    }
    SmartDashboard.putNumber("elevator height", getHeight());
  }

  public static void openClutch() {
    wasClutchEverOpen = true;
    RobotIO.clutchServo.set(RobotSettings.CLUTCH_OPEN_ANGLE);
    RobotIO.intakeArm.setIdleMode(IdleMode.kBrake);
  }

  public static void closeClutch() {
    RobotIO.clutchServo.set(RobotSettings.CLUTCH_CLOSE_ANGLE);
  }

  public static void set(double speed) {
    SmartDashboard.putBoolean("Was Clutch Open", wasClutchEverOpen);
    SmartDashboard.putNumber("ElevatorSpeed", speed);
    double height = getHeight();
    if ((height > RobotSettings.ELEVATOR_MAX_HEIGHT || wasClutchEverOpen || DriverStation.getInstance().getMatchTime() > 30) && speed > 0) {
      setSc(0);
      ElevatorSub.elevatorError = true;
      return;
    }
    if (height < 1 && speed < 0) {
      setSc(0);
      ElevatorSub.elevatorError = true;
      return;
    }
    setSc(speed);
    ElevatorSub.elevatorError = false;
  }

  public static double getrightHeight() {
    return rightEncoder.getPosition();
  }

  public static double getleftHeight() {
    return leftEncoder.getPosition();
  }

  public static double getHeight() {
    double result = Math.max(getleftHeight(), getrightHeight());
    if (result < 0) {
      rightEncoder.setPosition(0);
      leftEncoder.setPosition(0);
      return 0;
    }
    return result;
  }

  private static void setSc(double speed) {
    RobotIO.leftElevatorMotor.set(speed);
    RobotIO.rightElevatorMotor.set(-speed);
  }

}
