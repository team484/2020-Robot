/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.verticalconveyer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VerticalConveyer;

public class VerticalConveyorSetSpeed extends CommandBase {

  private double speed;
  public VerticalConveyorSetSpeed(VerticalConveyer subsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(subsystem);
    this.speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    VerticalConveyer.set(speed);
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