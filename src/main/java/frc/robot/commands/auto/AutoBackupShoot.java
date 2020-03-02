/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotSettings;
import frc.robot.commands.drivetrain.DriveUntilDistance;
import frc.robot.commands.drivetrain.RotateToTarget;
import frc.robot.commands.horizontalconveyor.HorizontalConveyorDoNothing;
import frc.robot.commands.horizontalconveyor.HorizontalConveyorSpin;
import frc.robot.commands.intakearm.IntakeArmToAngle;
import frc.robot.commands.shooter.PIDShooter;
import frc.robot.commands.shooter.ShooterWheelsDoNothing;
import frc.robot.commands.shooter.SpinShooterUntilRPM;
import frc.robot.commands.verticalconveyor.FeedWhenShooterReady;
import frc.robot.commands.verticalconveyor.VerticalConveyorRunWhenBall;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.HorizontalConveyorSub;
import frc.robot.subsystems.IntakeArmSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VerticalConveyer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoBackupShoot extends SequentialCommandGroup {
  /**
   * Creates a new AutoBackupShoot.
   */
  public AutoBackupShoot(DriveSub driveSub, IntakeArmSub intakeArmSub, IntakeSub intakeSub, HorizontalConveyorSub horizontalConveyerSub, VerticalConveyer verticalConveyer, ShooterSub shooterSub) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new DriveUntilDistance(-0.4, -1.5),
      //Now adjust robot to perfectly face target
      new ParallelRaceGroup(
        new RotateToTarget(driveSub),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new SpinShooterUntilRPM(shooterSub)
        )
      ),


      //FIRE
      new ParallelRaceGroup(
        new PIDShooter(shooterSub),
        //new RotateToTarget(driveSub),
        new HorizontalConveyorSpin(horizontalConveyerSub),
        new FeedWhenShooterReady(verticalConveyer),
        new WaitCommand(10)
      ),

      //Clean-up
      new ParallelCommandGroup(
        new IntakeArmToAngle(intakeArmSub, RobotSettings.INTAKE_UP_SETPOINT, true),
        new HorizontalConveyorDoNothing(horizontalConveyerSub),
        new VerticalConveyorRunWhenBall(verticalConveyer),
        new ShooterWheelsDoNothing(shooterSub)
      )

    );
  }
}
