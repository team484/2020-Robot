/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotSettings;
import frc.robot.Vision;
import frc.robot.subsystems.DriveSub;

public class RotateToTarget extends CommandBase {
  PIDController pid;
  /**
   * Creates a new RotateToTarget.
   */
  public RotateToTarget(DriveSub subsystem) {
    addRequirements(subsystem);
    pid = new PIDController(RobotSettings.DRIVE_ROTATE_KP,RobotSettings.DRIVE_ROTATE_KI,RobotSettings.DRIVE_ROTATE_KD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(0);
    pid.reset();
    Robot.driveSubVision = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rot = pid.calculate(Vision.getAngle());
    if (rot > 0.3) rot = 0.3;
    if (rot < -0.3) rot = -0.3;
    DriveSub.set(0, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSub.set(0, 0);
    Robot.driveSubVision = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
