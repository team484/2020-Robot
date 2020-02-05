/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;

public class VerticalConveyerSub extends SubsystemBase {
  /**
   * Creates a new VerticalConveyer.
   */
  public VerticalConveyerSub() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void conveyerUp(){
    RobotIO.ballConveyerVertical.set(1.0);
  }

  public static void conveyerDown(){
    RobotIO.ballConveyerVertical.set(-1.0);
  }
}
