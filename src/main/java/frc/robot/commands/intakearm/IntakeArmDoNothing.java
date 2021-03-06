/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intakearm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArmSub;

public class IntakeArmDoNothing extends CommandBase {
  public IntakeArmDoNothing(IntakeArmSub subsystem) {
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
    IntakeArmSub.setSpeed(0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public void end(boolean interrupted) {
    
  }

  // Called once after isFinished returns true
  @Override
  public boolean isFinished() {
    return false;
  }

}