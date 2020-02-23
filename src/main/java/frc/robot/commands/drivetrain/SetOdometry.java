/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSub;

public class SetOdometry extends CommandBase {
  double m_angle;
  Pose2d m_pose;
  /**
   * Creates a new SetOdometry.
   */
  public SetOdometry(DriveSub driveSub, Pose2d pose, double angle) {
    addRequirements(driveSub);
    m_angle = angle;
    m_pose = pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSub.setGyroAngle(m_angle);
    DriveSub.resetOdometry(m_pose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
