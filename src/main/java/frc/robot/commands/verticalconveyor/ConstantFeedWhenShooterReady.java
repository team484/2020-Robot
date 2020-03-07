/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.verticalconveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;
import frc.robot.Vision;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VerticalConveyer;

public class ConstantFeedWhenShooterReady extends CommandBase {
  private double rpm = 0;
  private double error = 0;
  private boolean feedLock = false;
  private int feedLockCounter = 0;

  public ConstantFeedWhenShooterReady(VerticalConveyer subsystem, double rpm, double error) {
    addRequirements(subsystem);
    this.rpm = rpm;
    this.error = error;
  }
  public ConstantFeedWhenShooterReady(VerticalConveyer subsystem, double rpm) {
    addRequirements(subsystem);
    this.rpm = rpm;
  }

  public ConstantFeedWhenShooterReady(VerticalConveyer subsystem) {
    addRequirements(subsystem);
    this.rpm = 0;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feedLock = false;
    feedLockCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] angDist = Vision.getAngleDistance();
    double errorAllowed = angDist[1]*(-0.05)+400.0;
    if (error > 0) {
      errorAllowed = error;
    }
    if (rpm == 0) {
      if (Math.abs(ShooterSub.getAveragedSpeed() - ShooterSub.getDesiredRPM()) < errorAllowed) {
        feedLockCounter++;
      }
    } else {
      if (Math.abs(ShooterSub.getAveragedSpeed() - rpm) < errorAllowed) {
        feedLockCounter++;
      }
    }

    if (feedLockCounter > 30) {
      feedLock = true;
    }

    if (feedLockCounter > 20) {
      VerticalConveyer.set(0.7);
    }

    if (feedLock) {
      //VerticalConveyer.set(errorAllowed/2000.0);
      RobotIO.shooterMotor3.config_kF(RobotSettings.SHOOTER_SLOT, RobotSettings.SHOOTER_KF *1.4, RobotSettings.CAN_TIMEOUT_INTERVAL);
      VerticalConveyer.set(0.7);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    VerticalConveyer.set(0);
    RobotIO.shooterMotor3.config_kF(RobotSettings.SHOOTER_SLOT, RobotSettings.SHOOTER_KF, RobotSettings.CAN_TIMEOUT_INTERVAL);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
