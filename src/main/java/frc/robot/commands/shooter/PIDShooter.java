/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision;
import frc.robot.subsystems.ShooterSub;

public class PIDShooter extends CommandBase {
  private double RPM;
  /**
   * Creates a new PIDShooter.
   */
  public PIDShooter(ShooterSub subsystem, double RPM) {
    addRequirements(subsystem);
    this.RPM = RPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rpm = Vision.getAngleDistance()[1]*10.968+8922.6;
    ShooterSub.setRPM(rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSub.setPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
