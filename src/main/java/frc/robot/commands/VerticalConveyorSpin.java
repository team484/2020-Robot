/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotIO;
import frc.robot.subsystems.VerticalConveyer;
import frc.robot.RobotSettings;

public class VerticalConveyorSpin extends CommandBase {

  /**
   * Creates a new JoystickDrive.
   */
  public VerticalConveyorSpin(VerticalConveyer subsystem) {
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
    VerticalConveyer.set(RobotSettings.VERTICAL_CONVEYOR_SPEED);
 }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    VerticalConveyer.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
