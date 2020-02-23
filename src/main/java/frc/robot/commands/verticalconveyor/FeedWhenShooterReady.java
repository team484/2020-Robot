/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.verticalconveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotSettings;
import frc.robot.Vision;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VerticalConveyer;

public class FeedWhenShooterReady extends CommandBase {
  private double rpm = 0;
  /**
   * Creates a new FeedWhenShooterReady.
   */
  public FeedWhenShooterReady(VerticalConveyer subsystem, double rpm) {
    addRequirements(subsystem);
    this.rpm = rpm;
  }

  public FeedWhenShooterReady(VerticalConveyer subsystem) {
    addRequirements(subsystem);
    this.rpm = 0;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] angDist = Vision.getAngleDistance();
    double errorAllowed = angDist[1]*(-0.1)+500.0;

    if (rpm == 0) {
      if (Math.abs(ShooterSub.getAveragedSpeed() - ShooterSub.getDesiredRPM()) < errorAllowed) {
        VerticalConveyer.set(1);
      } else {
        VerticalConveyer.set(0);
      }
    } else {
      if (Math.abs(ShooterSub.getAveragedSpeed() - rpm) < errorAllowed) {
        VerticalConveyer.set(1);
      } else {
        VerticalConveyer.set(0);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    VerticalConveyer.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
