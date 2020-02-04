/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotSettings;
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
    // This method will be called once per blah blah blah blah blah
  }
  public static void Set(double speed) {
    RobotIO.leftElevatorMotor.set(speed);
  }
  public static void engageClutch(){
    RobotIO.clutchServo.setAngle(RobotSettings.CLUTCH_ENGAGE_ANGLE);
  }
  public static void disengageClutch(){
    RobotIO.clutchServo.setAngle(RobotSettings.CLUTCH_DISENGAGE_ANGLE);
  }
  
}
