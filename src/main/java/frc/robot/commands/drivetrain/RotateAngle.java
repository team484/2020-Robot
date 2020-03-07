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
  private boolean curve = false;
  private int side = 1;
  /**
   * Creates a new RotateAngle.
   */
  private double angle;
  public RotateAngle(DriveSub subsystem, double angle) {
    addRequirements(subsystem);
    pid = new PIDController(RobotSettings.DRIVE_ROTATE_KP,RobotSettings.DRIVE_ROTATE_KI,RobotSettings.DRIVE_ROTATE_KD);
    this.angle = angle;
  }

  public RotateAngle(DriveSub subsystem, double angle, boolean curve) {
    addRequirements(subsystem);
    pid = new PIDController(RobotSettings.DRIVE_ROTATE_KP,RobotSettings.DRIVE_ROTATE_KI,RobotSettings.DRIVE_ROTATE_KD);
    this.angle = angle;
    this.curve = curve;
  }

  public RotateAngle(DriveSub subsystem, double angle, boolean curve, int side) {
    addRequirements(subsystem);
    pid = new PIDController(RobotSettings.DRIVE_ROTATE_KP,RobotSettings.DRIVE_ROTATE_KI,RobotSettings.DRIVE_ROTATE_KD);
    this.angle = angle;
    this.curve = curve;
    this.side = side;
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
    if (rot > 0.4) rot = 0.4;
    if (rot < -0.4) rot = -0.4;
    if (curve) {
      if (side == 1) {
        DriveSub.tankDriveWithVolts(-rot * 7, -rot);
      } else {
        DriveSub.tankDriveWithVolts(rot, rot * 7);
      }
    } else {
      DriveSub.set(0, rot);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSub.set(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(DriveSub.getGyroAngle() - angle) < 5;
  }
}
