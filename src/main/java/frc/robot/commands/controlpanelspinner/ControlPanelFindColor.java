/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controlpanelspinner;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotSettings;
import frc.robot.subsystems.ControlPanelSpinnerSub;
import frc.robot.subsystems.ControlPanelSpinnerSub.WheelColor;

public class ControlPanelFindColor extends CommandBase {
  int colorsSeenFor = 0;
  WheelColor desiredColor = WheelColor.UNKNOWN;
  /**
   * Creates a new ControlPanelFindColor.
   */
  public ControlPanelFindColor(ControlPanelSpinnerSub subsystem) {
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorsSeenFor = 0;
    desiredColor = getDesiredColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (desiredColor == WheelColor.UNKNOWN) {
      desiredColor = getDesiredColor();
      ControlPanelSpinnerSub.set(0);
      return;
    }
    WheelColor color = ControlPanelSpinnerSub.getColor();
    if (color == desiredColor) {
      colorsSeenFor++;
      ControlPanelSpinnerSub.set(0);
    } else {
      colorsSeenFor = 0;
      ControlPanelSpinnerSub.set(RobotSettings.CONTROL_PANEL_MOTOR_SPEED/2.0);
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
    return colorsSeenFor > 50; //Color has been seen for a second straight
  }

  private WheelColor getDesiredColor() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    switch (gameData.toLowerCase()) {
      case "y":
        return WheelColor.YELLOW;
      case "B":
        return WheelColor.BLUE;
      case "G":
        return WheelColor.GREEN;
      case "R":
        return WheelColor.RED;
      default:
        return WheelColor.UNKNOWN;
    }
  }
}
