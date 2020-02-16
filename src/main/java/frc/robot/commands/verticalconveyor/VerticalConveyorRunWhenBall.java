/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.verticalconveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotSettings;
import frc.robot.subsystems.HorizontalConveyorSub;
import frc.robot.subsystems.VerticalConveyer;

public class VerticalConveyorRunWhenBall extends CommandBase {
  /**
   * Creates a new VerticalConveyorRunWhenBall.
   */
  public VerticalConveyorRunWhenBall(VerticalConveyer subsystem) {
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (HorizontalConveyorSub.isBallPresent() && !VerticalConveyer.isBallPresent()) {
      VerticalConveyer.set(RobotSettings.VERTICAL_CONVEYOR_SPEED/2.0);
    } else {
      VerticalConveyer.set(0);
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