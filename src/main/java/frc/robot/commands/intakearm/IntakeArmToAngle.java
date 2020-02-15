/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intakearm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArmSub;

public class IntakeArmToAngle extends CommandBase {
  /**
   * Creates a new toAngle.
   */
   private double angle;
   private boolean exitWhenReached;
   public IntakeArmToAngle(IntakeArmSub subsystem, double angle, boolean exitWhenReached) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.angle = angle;
    this.exitWhenReached = exitWhenReached;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeArmSub.setAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return true if arm is at desired angle and exitWhenReached is true
    return false;
  }
}
