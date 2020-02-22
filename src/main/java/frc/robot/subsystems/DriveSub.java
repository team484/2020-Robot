/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU.IMUAxis;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;
import frc.robot.Vision;
import frc.robot.commands.drivetrain.JoystickDrive;

public class DriveSub extends SubsystemBase {
  public static DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(RobotSettings.DRIVEBASE_WIDTH);
  static DifferentialDriveOdometry driveOdometry;

  /**
   * Creates a new DriveSub.
   */
  public DriveSub() {
    setDefaultCommand(new JoystickDrive(this));
    RobotIO.leftMotor1.configFactoryDefault();
    RobotIO.leftMotor2.configFactoryDefault();
    RobotIO.leftMotor3.configFactoryDefault();
    RobotIO.rightMotor1.configFactoryDefault();
    RobotIO.rightMotor2.configFactoryDefault();
    RobotIO.rightMotor3.configFactoryDefault();

    RobotIO.rightMotor2.follow(RobotIO.rightMotor1);
    RobotIO.rightMotor3.follow(RobotIO.rightMotor1);

    RobotIO.leftMotor2.follow(RobotIO.leftMotor1);
    RobotIO.leftMotor3.follow(RobotIO.leftMotor1);
    RobotIO.leftMotor1.configMotorCommutation(MotorCommutation.Trapezoidal, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.leftMotor2.configMotorCommutation(MotorCommutation.Trapezoidal, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.leftMotor3.configMotorCommutation(MotorCommutation.Trapezoidal, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.rightMotor1.configMotorCommutation(MotorCommutation.Trapezoidal, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.rightMotor2.configMotorCommutation(MotorCommutation.Trapezoidal, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.rightMotor3.configMotorCommutation(MotorCommutation.Trapezoidal, RobotSettings.CAN_TIMEOUT_INTERVAL);
    setBrakeMode(true);
    RobotIO.leftEncoder.setDistancePerPulse(RobotSettings.LEFT_ENCODER_DPP);
    RobotIO.rightEncoder.setDistancePerPulse(RobotSettings.RIGHT_ENCODER_DPP);
    RobotIO.difDrive.setDeadband(0);
    driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroAngle()));

    RobotIO.imu.setYawAxis(IMUAxis.kY);
  }

  @Override
  public void periodic() {
    driveOdometry.update(Rotation2d.fromDegrees(getGyroAngle()), getLeftDistance(), getRightDistance());
    double[] angledist = Vision.getAngleDistance();
    SmartDashboard.putNumber("Target Angle", angledist[0]);
    SmartDashboard.putNumber("Target Dist", angledist[1]);
    SmartDashboard.putNumber("gyro", getGyroAngle());
    SmartDashboard.putNumber("Match time", DriverStation.getInstance().getMatchTime());
  }

  public static void set(double speed, double rot) {
    set(speed, rot, true);
  }

  public static void set(double speed, double rotation, boolean squareInputs) {
    RobotIO.difDrive.arcadeDrive(speed, -rotation, squareInputs);
  }

  public static void tankDrive(double leftSpeed, double rightSpeed) {
    RobotIO.difDrive.tankDrive(leftSpeed, rightSpeed, false);
  }

  public static void tankDriveWithVolts(double leftVolts, double rightVolts) {
    RobotIO.leftMotor1.setVoltage(leftVolts);
    RobotIO.rightMotor1.setVoltage(-rightVolts);
    RobotIO.difDrive.feed();
  }

  public static Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }

  public static void resetOdometry(Pose2d pose) {
    resetLeftDistance();
    resetRightDistance();
    driveOdometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));
  }

  public static double getLeftDistance() {
    return RobotIO.leftEncoder.getDistance();
  }

  public static double getLeftSpeed() {
    return RobotIO.leftEncoder.getRate();
  }

  public static void resetLeftDistance() {
    RobotIO.leftEncoder.reset();
  }

  public static double getRightDistance() {
    return RobotIO.rightEncoder.getDistance();
  }

  public static double getRightSpeed() {
    return RobotIO.rightEncoder.getRate();
  }

  public static void resetRightDistance() {
    RobotIO.rightEncoder.reset();
  }

  public static double getDistance() {
    return (getRightDistance() + getLeftDistance()) / 2;
  }

  public static DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }

  public static double getGyroAngle() {
    return RobotIO.imu.getAngle();
  }

  public static double getRotRate() {
    return -RobotIO.imu.getRate();
  }

  public static void resetAngle() {
    RobotIO.imu.reset();
  }

  public static double getLeftMotorVoltage() {
    return RobotIO.leftMotor1.getMotorOutputVoltage();
  }

  public static double getRightMotorVoltage() {
    return RobotIO.rightMotor1.getMotorOutputVoltage();
  }

  /**
   * Used to toggle voltage compensation in the drivetrain. This is a useful
   * feature when running PID loops or motion profiling.
   * 
   * @param enabled - Pass true to enable, false to disable
   * @param voltage - The voltage that equates to full power.
   */
  public static void setVoltageCompensation(boolean enabled, double voltage) {
    RobotIO.leftMotor1.configVoltageCompSaturation(voltage);
    RobotIO.leftMotor1.enableVoltageCompensation(enabled);
    RobotIO.leftMotor2.configVoltageCompSaturation(voltage);
    RobotIO.leftMotor2.enableVoltageCompensation(enabled);
    RobotIO.leftMotor3.configVoltageCompSaturation(voltage);
    RobotIO.leftMotor3.enableVoltageCompensation(enabled);

    RobotIO.rightMotor1.configVoltageCompSaturation(voltage);
    RobotIO.rightMotor1.enableVoltageCompensation(enabled);
    RobotIO.rightMotor2.configVoltageCompSaturation(voltage);
    RobotIO.rightMotor2.enableVoltageCompensation(enabled);
    RobotIO.rightMotor3.configVoltageCompSaturation(voltage);
    RobotIO.rightMotor3.enableVoltageCompensation(enabled);
  }

  public static void setBrakeMode(boolean brake) {
    RobotIO.leftMotor1.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    RobotIO.leftMotor2.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    RobotIO.leftMotor3.setNeutralMode(NeutralMode.Coast);
    RobotIO.rightMotor1.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    RobotIO.rightMotor2.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    RobotIO.rightMotor3.setNeutralMode(NeutralMode.Coast);
  }

}
