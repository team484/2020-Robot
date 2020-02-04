/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;
public class ShooterSub extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  public ShooterSub() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void shooterSpinWheels()
  {
    RobotIO.shooterMotor1.set(1);
    RobotIO.shooterMotor2.set(1);
  }  

}
