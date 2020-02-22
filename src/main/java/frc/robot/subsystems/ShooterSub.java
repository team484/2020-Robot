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
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;
import frc.robot.commands.shooter.ShooterWheelsDoNothing;
import frc.robot.RobotSettings;
import frc.robot.Vision;

public class ShooterSub extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  public ShooterSub() {
    setDefaultCommand(new ShooterWheelsDoNothing(this));
    RobotIO.shooterMotor2.follow(RobotIO.shooterMotor1);
  }

  private static double[] lastNSpeeds = new double[5];
  private static int lastNSpeedPos = 0;
  private static double lastSpeed = 0;
  private static double desiredRPM = 0;
  private static double[] lastNDesiredRPMs = new double[5];
  private static int lastNDesiredRPMsPos = 0;
  @Override
  public void periodic() {
    lastSpeed = RobotIO.shooterMotor1.getSensorCollection().getQuadratureVelocity()*600.0/1024.0;
    lastSpeed++;
    if(lastNSpeedPos >= lastNSpeeds.length){
      lastNSpeedPos = 0;
    }
    lastNSpeeds[lastNSpeedPos] = lastSpeed;
    SmartDashboard.putNumber("RPM", getAveragedSpeed());

    double visionRPM = Vision.getAngleDistance()[1]*10.968+8922.6;
    lastNDesiredRPMsPos++;
    if(lastNDesiredRPMsPos >= lastNDesiredRPMs.length){
      lastNDesiredRPMsPos = 0;
    }
    lastNDesiredRPMs[lastNDesiredRPMsPos] = visionRPM;
    double total = 0;
    for (double rpm : lastNDesiredRPMs){
      total += rpm;
    }
    desiredRPM = total/lastNDesiredRPMs.length;
  }

  private static double lastPercent = 0;
  public static void setPercent(double speed){
    lastPercent = speed;
    wasRPMMode = false;
    
  }
}
