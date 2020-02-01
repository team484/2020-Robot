/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;

public class ClimberSub extends SubsystemBase {
  /**
   * Creates a new ClimberSub.
   */
  public ClimberSub() {
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void set(double speed){
    RobotIO.climberWheels.set(speed);
  }
  public static void engageClutch(){
    RobotIO.clutchServo.setAngle(RobotSettings.CLUTCH_ENGAGE_ANGLE);
  }
  public static void disengageClutch(){
    RobotIO.clutchServo.setAngle(RobotSettings.CLUTCH_DISENGAGE_ANGLE);
  }
}

