/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterSub;

public class SpinShooterUntilRPM extends CommandBase {
  private double m_rpm = 0;
  /**
   * Creates a new SpinShooterUntilRPM.
   */
  public SpinShooterUntilRPM(ShooterSub subsystem, double rpm) {
    addRequirements(subsystem);
    m_rpm = rpm;
  }

  public SpinShooterUntilRPM(ShooterSub subsystem) {
    addRequirements(subsystem);
    m_rpm = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.shooterSubVision = true;
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShooterSub.setPercent(1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.shooterSubVision = false;
  }

  private double count = 0;
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double acceleration = (16000-ShooterSub.getInstantSpeed()) / 100;
    double speed = ShooterSub.getInstantSpeed() + acceleration;
    if (m_rpm > 0) {
      if (speed >= m_rpm) {
        count++;
      } else {
        count = 0;
      }
    }
    if (speed >= ShooterSub.getDesiredRPM()) {
      count++;
    } else {
      count = 0;
    }
    return (count > 3);
  }
}
