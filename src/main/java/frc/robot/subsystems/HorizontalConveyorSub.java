/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;
import frc.robot.commands.horizontalconveyer.HorizontalConveyorDoNothing;

public class HorizontalConveyorSub extends SubsystemBase {
  /**
   * Creates a new HorizontalConveyor.
   */
  public HorizontalConveyorSub() {
    setDefaultCommand(new HorizontalConveyorDoNothing(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
  public static void set(double speed){
    if (isBallIn() && VerticalConveyer.isBallIn()) {
      RobotIO.ballConveyerVertical.set(0);
      return;
    }
    RobotIO.ballConveyerVertical.set(speed);
  }

  public static boolean isBallIn() {
    return RobotIO.horizontalBallSensor.get();
  }
}
