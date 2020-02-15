/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;
import frc.robot.commands.shooter.ShooterWheelsDoNothing;
public class ShooterSub extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  public ShooterSub() {
    setDefaultCommand(new ShooterWheelsDoNothing(this));
    RobotIO.shooterMotor1.setInverted(InvertType.InvertMotorOutput);
    RobotIO.shooterMotor2.follow(RobotIO.shooterMotor1);
    RobotIO.shooterMotor2.setInverted(InvertType.FollowMaster);
    RobotIO.shooterMotor1.setNeutralMode(NeutralMode.Coast);
    RobotIO.shooterMotor2.setNeutralMode(NeutralMode.Coast);
    RobotIO.shooterMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.shooterMotor1.setSensorPhase(true);
    RobotIO.shooterMotor1.configPeakOutputForward(+1.0, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.shooterMotor1.configPeakOutputReverse(-1.0, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.shooterMotor1.configSelectedFeedbackCoefficient( 1.0/1024.0, 0, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.shooterMotor1.config_kP(RobotSettings.SHOOTER_SLOT, RobotSettings.SHOOTER_KP, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.shooterMotor1.config_kI(RobotSettings.SHOOTER_SLOT, RobotSettings.SHOOTER_KI, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.shooterMotor1.config_kD(RobotSettings.SHOOTER_SLOT, RobotSettings.SHOOTER_KD, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.shooterMotor1.config_kF(RobotSettings.SHOOTER_SLOT, RobotSettings.SHOOTER_KF, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.shooterMotor1.config_IntegralZone(RobotSettings.SHOOTER_SLOT, RobotSettings.SHOOTER_IZ, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.shooterMotor1.configAllowableClosedloopError(RobotSettings.SHOOTER_SLOT, 0, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.shooterMotor1.configClosedLoopPeriod(RobotSettings.SHOOTER_SLOT, 1, RobotSettings.CAN_TIMEOUT_INTERVAL);
    RobotIO.shooterMotor1.selectProfileSlot(RobotSettings.SHOOTER_SLOT, 0);

    RobotIO.shooterMotor1.enableCurrentLimit(true);
    RobotIO.shooterMotor1.configContinuousCurrentLimit(RobotSettings.SHOOTER_MAX_CONTINUOUS_CURRENT);
    RobotIO.shooterMotor1.configPeakCurrentLimit(RobotSettings.SHOOTER_MAX_PEAK_CURRENT);
    RobotIO.shooterMotor2.enableCurrentLimit(true);
    RobotIO.shooterMotor2.configContinuousCurrentLimit(RobotSettings.SHOOTER_MAX_CONTINUOUS_CURRENT);
    RobotIO.shooterMotor2.configPeakCurrentLimit(RobotSettings.SHOOTER_MAX_PEAK_CURRENT);


  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RPM", getSpeed());
  }

  public static void setPercent(double speed) {
    wasRPMMode = false;
    lastRPM = -1;
    RobotIO.shooterMotor1.set(ControlMode.PercentOutput, speed);
  }  

  static double lastRPM = -1;
  static boolean wasRPMMode = false;
  public static void setRPM(double rpm) {
    if (!wasRPMMode || lastRPM != rpm) {
      wasRPMMode = true;
      lastRPM = rpm;
   RobotIO.shooterMotor1.set(ControlMode.Velocity, rpm / 600.0);
    }
  }

  public static double getSpeed() {
    return RobotIO.shooterMotor1.getSensorCollection().getQuadratureVelocity()*600.0/1024.0;
  }

}
