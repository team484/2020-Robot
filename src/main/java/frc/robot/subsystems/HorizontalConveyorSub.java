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
import frc.robot.commands.horizontalconveyor.HorizontalConveyorDoNothing;

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
    if (isBallPresent() && VerticalConveyer.isBallPresent()) {
      RobotIO.ballConveyerHorizontal.set(0);
      return;
    }
    RobotIO.ballConveyerHorizontal.set(speed);
  }

  public static boolean isBallPresent() {
    return RobotIO.horizontalBallSensor.get() == RobotSettings.HORIZONTAL_SENSOR_NORMAL_STATE;
  }
}
