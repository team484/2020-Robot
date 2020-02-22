/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controlpanel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotSettings;
import frc.robot.subsystems.ControlPanelSpinnerSub;
import frc.robot.subsystems.ControlPanelSpinnerSub.WheelColor;

public class ControlPanelSpinnerSpinTo extends CommandBase {
  WheelColor desired = WheelColor.UNKNOWN;
  int timeSeen = 0;
  /**
   * Creates a new ControlPanelSpinnerSpinTo.
   */
  public ControlPanelSpinnerSpinTo(ControlPanelSpinnerSub subsystem) {
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled .
  @Override
  public void initialize() {
    desired = receiveColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (desired == WheelColor.UNKNOWN) {
      ControlPanelSpinnerSub.set(0);
      return;
    } else if (desired == ControlPanelSpinnerSub.getColor()) {
      timeSeen++;
      ControlPanelSpinnerSub.set(0);
    } else {
      ControlPanelSpinnerSub.set(RobotSettings.CONTROL_PANEL_MOTOR_SPEED/2.0);
      timeSeen = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ControlPanelSpinnerSub.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeSeen > 100;
  }

  public static WheelColor receiveColor() {
    String gameMessage = DriverStation.getInstance().getGameSpecificMessage();
    switch (gameMessage.toLowerCase()) {
      case "r":
        return WheelColor.RED;
      case "g":
        return WheelColor.GREEN;
      case "b":
        return WheelColor.BLUE;
      case "y":
        return WheelColor.YELLOW;
      default:
        return WheelColor.UNKNOWN;
    }
  }

}
