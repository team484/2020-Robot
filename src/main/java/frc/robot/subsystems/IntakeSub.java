/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;
import frc.robot.commands.IntakeDoNothing;

public class IntakeSub extends SubsystemBase {
  /**
   * Creates a new IntakeSub.
   */
  public IntakeSub() {
    setDefaultCommand(new IntakeDoNothing(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void intakeSpinWheels(double speed) {
    RobotIO.intakeAndControl.set(speed);
  }
  
}
