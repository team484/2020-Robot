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
import com.ctre.phoenix.motorcontrol.StatusFrame;

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
    RobotIO.shooterMotor1.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotSettings.CAN_TIMEOUT_INTERVAL);
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

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void setPercent(double speed) {
    RobotIO.shooterMotor1.set(ControlMode.PercentOutput, speed);
  }  

  public static void setRPM(double rpm) {
    RobotIO.shooterMotor1.set(ControlMode.Velocity, rpm / 600.0);
  }

  public static double getSpeed() {
    return RobotIO.shooterMotor1.getSensorCollection().getQuadratureVelocity()*600.0;
  }

}
