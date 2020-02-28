/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.verticalconveyer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VerticalConveyer;
import frc.robot.RobotSettings;
import frc.robot.subsystems.HorizontalConveyorSub;

public class VerticalConveyorRunWhenBall extends CommandBase {
  public VerticalConveyorRunWhenBall(VerticalConveyer subsystem) {
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(subsystem);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if(HorizontalConveyorSub.isBallIn() && !VerticalConveyer.isBallIn())
      VerticalConveyer.set(RobotSettings.VERTICAL_CONVEYOR_SPEED);
    else
      VerticalConveyer.set(0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public void end(boolean interrupted) {
    VerticalConveyer.set(0);
  }

  // Called once after isFinished returns true
  @Override
  public boolean isFinished() {
    return false;
  }

}