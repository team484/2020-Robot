/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;
/**
 * Add your docs here.
 */
public class IntakeArmSub extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public IntakeArmSub() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void intakeLower() {
    RobotIO.intakeArm.set(1.0);
  }

  public static void intakeRaise() {
    RobotIO.intakeArm.set(-1.0);
  }
  
}
