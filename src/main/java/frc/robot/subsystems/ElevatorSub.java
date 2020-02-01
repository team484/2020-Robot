/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;

public class ElevatorSub extends SubsystemBase {
  /**
   * Creates a new Elevator.
   */
  public ElevatorSub() {
    RobotIO.rightElevatorMotor.follow(RobotIO.leftElevatorMotor, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public static void Set(double speed) {
    RobotIO.leftElevatorMotor.setVoltage(speed);
  }
  public static void engageClutch(){
    
  }
}
