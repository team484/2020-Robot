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
import frc.robot.RobotContainer;
import frc.robot.RobotSettings;
import frc.robot.commands.drivetrain.RotateAngle;
import frc.robot.commands.drivetrain.RotateToTarget;
import frc.robot.commands.horizontalconveyor.HorizontalConveyorDoNothing;
import frc.robot.commands.horizontalconveyor.HorizontalConveyorSpin;
import frc.robot.commands.intake.IntakeSpin;
import frc.robot.commands.intakearm.IntakeArmSetPower;
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
public class AutoTrench extends SequentialCommandGroup {
  /**
   * Creates a new AutoTrench.
   */
  public AutoTrench(DriveSub driveSub, IntakeArmSub intakeArmSub, IntakeSub intakeSub, HorizontalConveyorSub horizontalConveyerSub, VerticalConveyer verticalConveyer, ShooterSub shooterSub) {
    super(

    //Drive over and pick up balls
      new ParallelRaceGroup(
        RobotContainer.generateTrajectoryCommand("trench", driveSub),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new ParallelCommandGroup(
            new IntakeArmToAngle(intakeArmSub, RobotSettings.INTAKE_DOWN_SETPOINT, false),
            new IntakeSpin(intakeSub, RobotSettings.INTAKE_WHEELS_MOTOR_SPEED),
            new HorizontalConveyorSpin(horizontalConveyerSub)
          )
        )
      ),

      //Sit there for another second after driving to make sure you get all the balls
      new ParallelRaceGroup(
        new IntakeArmSetPower(intakeArmSub, RobotSettings.INTAKE_ARM_HORIZ_HOLD_POWER),
        new IntakeSpin(intakeSub, RobotSettings.INTAKE_WHEELS_MOTOR_SPEED),
        new HorizontalConveyorSpin(horizontalConveyerSub),
        new WaitCommand(1)
      ),

      //Now rotate -160 degrees to face target
      new ParallelRaceGroup(
        new RotateAngle(driveSub, -160),
        new WaitCommand(2.0) //cancel after 2 seconds
      ),


      //Now adjust robot to perfectly face target
      new ParallelRaceGroup(
        new RotateToTarget(driveSub),
        new WaitCommand(3.0)
      ),

      //Spin up wheels
      new SpinShooterUntilRPM(shooterSub),

      //FIRE
      new ParallelRaceGroup(
        new PIDShooter(shooterSub),
        new HorizontalConveyorSpin(horizontalConveyerSub),
        new FeedWhenShooterReady(verticalConveyer),
        new WaitCommand(7)
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