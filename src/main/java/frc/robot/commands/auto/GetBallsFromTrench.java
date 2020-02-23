/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class GetBallsFromTrench extends ParallelCommandGroup {
  /**
   * Creates a new GetBallsFromTrench.
   */
  public GetBallsFromTrench(DriveSub driveSub) {
    // Add your commands in the super() call, e.g.
  }
}
