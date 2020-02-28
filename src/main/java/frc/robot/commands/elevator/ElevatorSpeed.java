/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotSettings;
import frc.robot.subsystems.ElevatorSub;

public class ElevatorSpeed extends CommandBase {
  private double speed;
  /**
   * Creates a new ElevatorSpeed.
   */
  public ElevatorSpeed(ElevatorSub elevatorSub, double speed) {
    addRequirements(elevatorSub);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dist = Math.min(ElevatorSub.getHeight(), RobotSettings.ELEVATOR_MAX_HEIGHT-ElevatorSub.getHeight());

    ElevatorSub.set(Math.min(speed, speed * dist / 15.0 + 0.1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ElevatorSub.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
