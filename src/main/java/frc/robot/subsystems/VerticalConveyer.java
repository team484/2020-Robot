/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;
import frc.robot.commands.verticalconveyor.VerticalConveyorRunWhenBall;

public class VerticalConveyer extends SubsystemBase {
  /**
   * Creates a new VerticalConveyer.
   */
  public VerticalConveyer() {
    setDefaultCommand(new VerticalConveyorRunWhenBall(this));
    RobotIO.ballConveyerVertical.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void set(double speed){
    RobotIO.ballConveyerVertical.set(speed);
  }

  public static boolean isBallPresent() {
    return RobotIO.verticalBallSensor.get() != RobotSettings.VERTICAL_SENSOR_NORMAL_STATE;
  }
  
}
