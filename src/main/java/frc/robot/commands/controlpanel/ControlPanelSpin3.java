/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controlpanel;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotIO;
import frc.robot.subsystems.ControlPanelSpinnerSub;
import frc.robot.subsystems.ControlPanelSpinnerSub.WheelColor;

public class ControlPanelSpin3 extends CommandBase {
  WheelColor startColor = WheelColor.UNKNOWN;
  boolean flip = false;
  int hits = 0;
  /**
   * Creates a new ControlPanelSpin3.
   */
  public ControlPanelSpin3(ControlPanelSpinnerSub subsystem) {
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled .
  @Override
  public void initialize() {
    startColor = ControlPanelSpinnerSub.getColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ControlPanelSpinnerSub.set(0.5);
    if (ControlPanelSpinnerSub.getColor() == startColor) {
      if (flip)
        hits++;
      flip = false;
    } else
      flip = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ControlPanelSpinnerSub.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hits >= 6;
  }
}
