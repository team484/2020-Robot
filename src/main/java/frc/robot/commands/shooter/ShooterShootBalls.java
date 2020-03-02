/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShooterShootBalls extends SequentialCommandGroup {
  /**
   * Creates a new ShooterShootBalls.
   */
  public ShooterShootBalls(ShooterSub shooterSub) {
    super(new SpinShooterUntilRPM(shooterSub), new PIDShooter(shooterSub));
  }

  public ShooterShootBalls(ShooterSub shooterSub, double rpm) {
    super(new SpinShooterUntilRPM(shooterSub, rpm), new PIDShooter(shooterSub, rpm));
  }
}
