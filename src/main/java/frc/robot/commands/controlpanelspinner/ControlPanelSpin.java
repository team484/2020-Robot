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

public class ControlPanelSpin extends CommandBase {
  /**
   * Creates a new ControlPanelSpin.
   */
  public double controlPanelSpin = RobotSettings.CONTROL_PANEL_MOTOR_SPEED;

  public ControlPanelSpin(ControlPanelSpinnerSub subsystem, double speed) {
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ControlPanelSpinnerSub.set(controlPanelSpin);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ControlPanelSpinnerSub.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
