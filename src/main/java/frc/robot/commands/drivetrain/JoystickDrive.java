/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;
import frc.robot.subsystems.DriveSub;

public class JoystickDrive extends CommandBase {

  /**
   * Creates a new JoystickDrive.
   */
  public JoystickDrive(DriveSub subsystem) {
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute()
 {
  DriveSub.set(-RobotIO.driveStick.getY(), -RobotIO.driveStick.getX() * RobotSettings.JOYSTICK_ROTATION_MULTIPLIER);
 }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSub.set(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}