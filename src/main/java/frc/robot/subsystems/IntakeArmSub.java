/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;
import frc.robot.commands.IntakeArmDoNothing;
/**
 * Add your docs here.
 */
public class IntakeArmSub extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public IntakeArmSub() {
    setDefaultCommand(new IntakeArmDoNothing(this));
    RobotIO.intakeArmPID.setP(RobotSettings.INTAKE_KP);
    RobotIO.intakeArmPID.setI(RobotSettings.INTAKE_KI);
    RobotIO.intakeArmPID.setD(RobotSettings.INTAKE_KD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void setAngle(double angle){
    RobotIO.intakeArmPID.setReference(angle, ControlType.kPosition);
  }
  private static double lastSpeed = -2;
  public static void setSpeed(double speed)
  {
    if (speed == lastSpeed) return;
    lastSpeed = speed;
    RobotIO.intakeArmPID.setReference(speed, ControlType.kDutyCycle);
    RobotIO.intakeArm.set(speed);
  }
  
}
