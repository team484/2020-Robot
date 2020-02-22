/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;
import frc.robot.commands.intakearm.IntakeArmSetPower;
/**
 * Add your docs here.
 */
public class IntakeArmSub extends SubsystemBase {
  private static boolean lastRunSpeed = true;
  private static double lastAngle = -1;
  private static double lastSpeed = -2;
  private static CANEncoder encoder;

  public IntakeArmSub() {
    setDefaultCommand(new IntakeArmSetPower(this,RobotSettings.INTAKE_ARM_VERT_HOLD_POWER));
    RobotIO.intakeArmPID.setP(RobotSettings.INTAKE_KP);
    RobotIO.intakeArmPID.setI(RobotSettings.INTAKE_KI);
    RobotIO.intakeArmPID.setD(RobotSettings.INTAKE_KD);
    RobotIO.intakeArmPID.setOutputRange(-0.3, 0.3);
    RobotIO.intakeArm.setSmartCurrentLimit(RobotSettings.INTAKE_ARM_MAX_CURRENT);
    encoder = RobotIO.intakeArm.getEncoder();
  }

  @Override
  public void periodic() {
    if (getAngle() > 0) {
      encoder.setPosition(0);
    }
    SmartDashboard.putNumber("Intake Arm Ang", getAngle());
  }

  public static void setAngle(double angle){
    if (angle == lastAngle && !lastRunSpeed) return;
    lastRunSpeed = false;
    RobotIO.intakeArmPID.setReference(angle, ControlType.kPosition);
  }

  public static void setSpeed(double speed) {
    if (speed == lastSpeed && lastRunSpeed) return;
    lastSpeed = speed;
    lastRunSpeed = true;
    RobotIO.intakeArmPID.setReference(speed, ControlType.kDutyCycle);
    RobotIO.intakeArm.set(speed);
  }

  public static double getAngle() {
    return encoder.getPosition();
  }

  public static double getVelocity() {
    return encoder.getVelocity();
  }
  
}
