/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//WPILib Imports
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.DoNothing;
import frc.robot.commands.auto.DriveExamplePath;
import frc.robot.commands.climber.*;
import frc.robot.commands.controlpanelspinner.ControlPanelFindColor;
import frc.robot.commands.controlpanelspinner.ControlPanelRotate3x;
import frc.robot.commands.controlpanelspinner.ControlPanelSpin;
import frc.robot.commands.controlpanelspinner.ControlPanelSpinDoNothing;
import frc.robot.commands.drivetrain.CharacterizeDrivetrain;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.RotateToTarget;
import frc.robot.commands.elevator.*;
import frc.robot.commands.horizontalconveyor.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.intakearm.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.verticalconveyor.FeedWhenShooterReady;
import frc.robot.commands.verticalconveyor.VerticalConveyorRunWhenBall;
//Subsystem Imports
import frc.robot.subsystems.ClimberSub;
import frc.robot.subsystems.ControlPanelSpinnerSub;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.HorizontalConveyorSub;
import frc.robot.subsystems.IntakeArmSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VerticalConveyer;




/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ClimberSub climberSub = new ClimberSub();
  private final ControlPanelSpinnerSub controlPanelSpinner = new ControlPanelSpinnerSub();
  private final DriveSub driveSub = new DriveSub();
  private final ElevatorSub elevatorSub = new ElevatorSub();
  private final HorizontalConveyorSub horizontalConveyerSub = new HorizontalConveyorSub();
  private final IntakeArmSub intakeArmSub = new IntakeArmSub();
  private final IntakeSub intakeSub = new IntakeSub();
  private final ShooterSub shooterSub = new ShooterSub();
  private final VerticalConveyer verticalConveyerSub = new VerticalConveyer();

  public static final Vision vision = new Vision();
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    autoChooser.setDefaultOption("Do Nothing", new DoNothing());
    autoChooser.addOption("Example Auto", new DriveExamplePath(driveSub));
    autoChooser.addOption("Characterize Drivetrain", new CharacterizeDrivetrain(driveSub));
    SmartDashboard.putData(autoChooser);
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoChooser.getSelected() != null) {
      return autoChooser.getSelected();
    } else {
      return new DoNothing();
    }
  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Driver Commands
    //-----control elevator-----
    new JoystickButton(RobotIO.driveStick, RobotSettings.ELEVATOR_CONTROLS_BUTTON)
    .whileHeld(new JoystickElevator(elevatorSub, driveSub))
    .whileHeld(new JoystickClimber(climberSub))
    .whenReleased(new JoystickDrive(driveSub))
    .whenReleased(new ElevatorDoNothing(elevatorSub))
    .whenReleased(new ClimberDoNothing(climberSub));

    //-----open clutch-----
    new JoystickButton(RobotIO.driveStick, RobotSettings.OPEN_CLUTCH_BUTTON)
    .whenPressed(new OpenClutch());

    //-----Driver Aim-----
    new JoystickButton(RobotIO.driveStick, RobotSettings.DRIVER_AIM_BUTTON)
    .whenPressed(new RotateToTarget(driveSub))
    .whenReleased(new JoystickDrive(driveSub));
    
    //Operator Commands
    //-----shoot ball-----
    new JoystickButton(RobotIO.operatorStick, RobotSettings.BALL_SHOOTER_BUTTON)
    .whenPressed(new PIDShooter(shooterSub, RobotSettings.SHOOTER_TARGET_RPM))
    .whenPressed(new FeedWhenShooterReady(verticalConveyerSub, RobotSettings.SHOOTER_TARGET_RPM))
    .whenReleased(new ShooterWheelsDoNothing(shooterSub))
    .whenReleased(new VerticalConveyorRunWhenBall(verticalConveyerSub));

    //-----pickup ball (automatic ball intake)-----
    new JoystickButton(RobotIO.operatorStick, RobotSettings.PICKUP_BALL_BUTTON)
    .whenPressed(new IntakeSpin(intakeSub, RobotSettings.INTAKE_WHEELS_MOTOR_SPEED))
    .whenPressed(new HorizontalConveyorSpin(horizontalConveyerSub))
    .whenPressed((new IntakeArmToAngle(intakeArmSub, RobotSettings.INTAKE_DOWN_SETPOINT, true)
    .andThen(new IntakeArmSetPower(intakeArmSub, RobotSettings.INTAKE_ARM_HORIZ_HOLD_POWER))))
    .whenReleased(new IntakeArmToAngle(intakeArmSub, RobotSettings.INTAKE_UP_SETPOINT, true))
    .whenReleased((new WaitCommand(1.5))
    .andThen(new IntakeDoNothing(intakeSub)))
    .whenReleased((new WaitCommand(4.0))
    .andThen(new HorizontalConveyorDoNothing(horizontalConveyerSub)));

    //-----lower intake-----
    new JoystickButton(RobotIO.operatorStick, RobotSettings.INTAKE_LOWER_LEVEL_BUTTON)
    .whenPressed(new IntakeArmToAngle(intakeArmSub, RobotSettings.INTAKE_DOWN_SETPOINT, true)
    .andThen(new IntakeArmSetPower(intakeArmSub, RobotSettings.INTAKE_ARM_HORIZ_HOLD_POWER)));

    //-----raise intake-----
    new JoystickButton(RobotIO.operatorStick, RobotSettings.INTAKE_UPPER_LEVEL_BUTTON)
    .whenPressed(new IntakeArmToAngle(intakeArmSub, RobotSettings.INTAKE_UP_SETPOINT, true));

    //-----middle height intake-----
    new JoystickButton(RobotIO.operatorStick, RobotSettings.INTAKE_MIDDLE_LEVEL_BUTTON)
    .whenPressed(new IntakeArmToAngle(intakeArmSub, RobotSettings.INTAKE_MID_SETPOINT, true));

    //-----manual spin (ctrl panel/intake/horizontalconveyor)
    new JoystickButton(RobotIO.operatorStick, RobotSettings.INTAKE_WHEELS_AND_CONTROL_SPIN_BUTTON)
    .whileHeld(new IntakeSpin(intakeSub, RobotSettings.INTAKE_WHEELS_MOTOR_SPEED))
    .whileHeld(new HorizontalConveyorSpin(horizontalConveyerSub))
    .whileHeld(new ControlPanelSpin(controlPanelSpinner, RobotSettings.CONTROL_PANEL_MOTOR_SPEED))
    .whenReleased(new IntakeDoNothing(intakeSub))
    .whenReleased(new HorizontalConveyorDoNothing(horizontalConveyerSub))
    .whenReleased(new ControlPanelSpinDoNothing(controlPanelSpinner));

    //-----Rotate Control Panel 3 times-----
    new JoystickButton(RobotIO.operatorStick, RobotSettings.ROTATION_CONTROL_BUTTON)
    .whenPressed(new ControlPanelRotate3x(controlPanelSpinner))
    .whenReleased(new ControlPanelSpinDoNothing(controlPanelSpinner));

    //-----Rotate Control Panel to color-----
    new JoystickButton(RobotIO.operatorStick, RobotSettings.POSITION_CONTROL_BUTTON)
    .whenPressed(new ControlPanelFindColor(controlPanelSpinner))
    .whenReleased(new ControlPanelSpinDoNothing(controlPanelSpinner));
  }




    /**
   * Generates the command which will drive the desired trajectory.
   * Maybe one of these days I'll add comments to this
   * @param trajectoryName - the name of the trajectory to run
   * @param driveSub - the drivetrain's subsystem
   * @return - the command to execute for the trajectory
   */
  public static Command generateTrajectoryCommand(String trajectoryName, DriveSub driveSub) {
    String trajectoryJSON = "output/"+trajectoryName+".wpilib.json";
    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, e.getStackTrace());
      return new WaitCommand(0); // Command to do nothing
    }
    
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory, 
      DriveSub::getPose, 
      new RamseteController(RobotSettings.DRIVE_RAMSETE_B, RobotSettings.DRIVE_RAMSETE_Z),
      new SimpleMotorFeedforward(
        RobotSettings.DRIVE_KS, 
        RobotSettings.DRIVE_KV,
        RobotSettings.DRIVE_KA),
      DriveSub.driveKinematics,
      DriveSub::getWheelSpeeds,
      new PIDController(RobotSettings.DRIVE_KP, 0, 0), 
      new PIDController(RobotSettings.DRIVE_KP, 0, 0),
      DriveSub::tankDriveWithVolts,
      driveSub
      );

      return ramseteCommand.andThen(() -> DriveSub.tankDrive(0, 0));
  }
}