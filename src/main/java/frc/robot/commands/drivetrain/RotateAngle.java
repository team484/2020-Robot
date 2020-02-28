/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotSettings;
import frc.robot.subsystems.DriveSub;

public class RotateAngle extends CommandBase {
  PIDController pid;
  /**
   * Creates a new RotateAngle.
   */
  private double angle;
  public RotateAngle(DriveSub subsystem, double angle) {
    addRequirements(subsystem);
    pid = new PIDController(RobotSettings.DRIVE_ROTATE_KP,RobotSettings.DRIVE_ROTATE_KI,RobotSettings.DRIVE_ROTATE_KD);
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(angle);
    pid.reset();
    DriveSub.resetAngle();
    DriveSub.setGyroAngle(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rot = pid.calculate(DriveSub.getGyroAngle());
    if (rot > 0.38) rot = 0.38;
    if (rot < -0.38) rot = -0.38;
    DriveSub.set(0, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSub.set(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
