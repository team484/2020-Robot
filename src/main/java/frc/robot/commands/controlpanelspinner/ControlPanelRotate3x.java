/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controlpanelspinner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotSettings;
import frc.robot.subsystems.ControlPanelSpinnerSub;
import frc.robot.subsystems.ControlPanelSpinnerSub.WheelColor;

public class ControlPanelRotate3x extends CommandBase {
  WheelColor lastSeenColor = WheelColor.UNKNOWN;
  int colorsSeen = 0;
  /**
   * Creates a new ControlPanelRotate3x.
   */
  public ControlPanelRotate3x(ControlPanelSpinnerSub subsystem) {
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastSeenColor = ControlPanelSpinnerSub.getColor();
    colorsSeen = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ControlPanelSpinnerSub.set(RobotSettings.CONTROL_PANEL_MOTOR_SPEED);

    if (lastSeenColor == WheelColor.UNKNOWN) {
      lastSeenColor = ControlPanelSpinnerSub.getColor();
      // Spin slowly while waiting to see a color
      return;
    }
    WheelColor newColor = ControlPanelSpinnerSub.getColor();
    if (newColor == lastSeenColor || newColor == WheelColor.UNKNOWN) {
      return;
    }
    // Count different colors seen. 8 colors per rotation
    colorsSeen++;
    lastSeenColor = newColor;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ControlPanelSpinnerSub.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return colorsSeen > 8*3;
  }
}
