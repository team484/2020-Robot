/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controlpanelspinner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSpinnerSub;
import frc.robot.subsystems.IntakeSub;

public class ControlPanelSpinnerTest extends CommandBase {
  private int testStage = 0;
  /**
   * Creates a new ControlPanelSpinnerTest.
   */
  public ControlPanelSpinnerTest(ControlPanelSpinnerSub cpsSub, IntakeSub iSub) {
    addRequirements(cpsSub,iSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    testStage = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
