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
import frc.robot.commands.auto.*;
import frc.robot.commands.controlpanelspinner.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.elevator.*;
import frc.robot.commands.horizontalconveyor.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.intakearm.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.verticalconveyor.*;
//Subsystem Imports
import frc.robot.subsystems.ControlPanelSpinnerSub;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.HorizontalConveyorSub;
import frc.robot.subsystems.IntakeArmSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LEDSub;
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
  private ControlPanelSpinnerSub controlPanelSpinner;
  private DriveSub driveSub;
  private ElevatorSub elevatorSub;
  private HorizontalConveyorSub horizontalConveyerSub;
  private IntakeArmSub intakeArmSub;
  private IntakeSub intakeSub;
  private ShooterSub shooterSub;
  private VerticalConveyer verticalConveyerSub;
  private LEDSub ledSub;

  public static final Vision vision = new Vision();
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    try {
      controlPanelSpinner = new ControlPanelSpinnerSub();
      Thread.sleep(50);
      driveSub = new DriveSub();
      Thread.sleep(50);
      elevatorSub = new ElevatorSub();
      Thread.sleep(50);
      horizontalConveyerSub = new HorizontalConveyorSub();
      Thread.sleep(50);
      intakeArmSub = new IntakeArmSub();
      Thread.sleep(50);
      intakeSub = new IntakeSub();
      Thread.sleep(50);
      shooterSub = new ShooterSub();
      Thread.sleep(50);
      verticalConveyerSub = new VerticalConveyer();
      Thread.sleep(50);
      ledSub = new LEDSub();
    } catch (Exception e) {
      return;
    }
    // Configure the button bindings
    configureButtonBindings();
    autoChooser.setDefaultOption("Do Nothing", new DoNothing());
    autoChooser.addOption("Example Auto", new DriveExamplePath(driveSub));
    autoChooser.addOption("Characterize Drivetrain", new CharacterizeDrivetrain(driveSub));
    autoChooser.addOption("(1) Trench", new AutoTrench(driveSub, intakeArmSub, intakeSub, horizontalConveyerSub, verticalConveyerSub, shooterSub));
    autoChooser.addOption("(2) Shield Generator", new AutoShield(driveSub, intakeArmSub, intakeSub, horizontalConveyerSub, verticalConveyerSub, shooterSub));
    autoChooser.addOption("(3) Back up and shoot", new AutoBackupShoot(driveSub, intakeArmSub, intakeSub, horizontalConveyerSub, verticalConveyerSub, shooterSub));
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
    .whenReleased(new JoystickDrive(driveSub))
    .whenReleased(new ElevatorDoNothing(elevatorSub));

    //-----open clutch-----
    new JoystickButton(RobotIO.driveStick, RobotSettings.OPEN_CLUTCH_BUTTON)
    .whenPressed(new OpenClutch());

    //-----Driver Aim-----
    new JoystickButton(RobotIO.driveStick, RobotSettings.DRIVER_AIM_BUTTON)
    .whenPressed(new RotateToTarget(driveSub))
    .whenReleased(new JoystickDrive(driveSub));

    //-----ELEVATOR UP-----
    new JoystickButton(RobotIO.driveStick, RobotSettings.ELEVATOR_UP_BUTTON)
    .whenPressed(new ElevatorSpeed(elevatorSub, 1.0))
    .whenReleased(new ElevatorDoNothing(elevatorSub));

    //-----ELEVATOR DOWN-----
    new JoystickButton(RobotIO.driveStick, RobotSettings.ELEVATOR_DOWN_BUTTON)
    .whenPressed(new ElevatorSpeed(elevatorSub, -0.5))
    .whenReleased(new ElevatorDoNothing(elevatorSub));

    //-----shoot ball far-----
    new JoystickButton(RobotIO.operatorStick, RobotSettings.POSITION_CONTROL_BUTTON)
    .whenPressed(new ShooterShootBalls(shooterSub, 13300))
    .whenPressed(new ConstantFeedWhenShooterReady(verticalConveyerSub, 13300)) //.whenPressed(new FeedWhenShooterReady(verticalConveyerSub))
    .whenPressed(new HorizontalConveyorSpin(horizontalConveyerSub))
    .whenReleased(new ShooterWheelsDoNothing(shooterSub))
    .whenReleased(new VerticalConveyorRunWhenBall(verticalConveyerSub))
    .whenReleased(new HorizontalConveyorDoNothing(horizontalConveyerSub));


    //Operator Commands
    //-----shoot ball-----
    new JoystickButton(RobotIO.operatorStick, RobotSettings.BALL_SHOOTER_BUTTON)
    .whenPressed(new ShooterShootBalls(shooterSub))
    .whenPressed(new ConstantFeedWhenShooterReady(verticalConveyerSub)) //.whenPressed(new FeedWhenShooterReady(verticalConveyerSub))
    .whenPressed(new HorizontalConveyorSpin(horizontalConveyerSub))
    .whenReleased(new ShooterWheelsDoNothing(shooterSub))
    .whenReleased(new VerticalConveyorRunWhenBall(verticalConveyerSub))
    .whenReleased(new HorizontalConveyorDoNothing(horizontalConveyerSub));


    //-----pickup ball (automatic ball intake)-----
    new JoystickButton(RobotIO.operatorStick, RobotSettings.PICKUP_BALL_BUTTON)
    .whenPressed(new IntakeSpin(intakeSub, RobotSettings.INTAKE_WHEELS_MOTOR_SPEED))
    .whenPressed(new HorizontalConveyorSpin(horizontalConveyerSub))
    .whenPressed((new IntakeArmToAngle(intakeArmSub, RobotSettings.INTAKE_DOWN_SETPOINT, true)
    .andThen(new IntakeArmSetPower(intakeArmSub, RobotSettings.INTAKE_ARM_HORIZ_HOLD_POWER))))
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
    /*new JoystickButton(RobotIO.operatorStick, RobotSettings.POSITION_CONTROL_BUTTON)
    .whenPressed(new ControlPanelFindColor(controlPanelSpinner))
    .whenReleased(new ControlPanelSpinDoNothing(controlPanelSpinner));*/

    //-----EJECT BALLS-----
    new JoystickButton(RobotIO.operatorStick, RobotSettings.EJECT_BUTTON)
    .whileHeld(new HorizontalConveyorSpin(horizontalConveyerSub, -1))
    .whileHeld(new VerticalConveyorSpin(verticalConveyerSub, -0.3))
    .whileHeld(new IntakeSpin(intakeSub, -1))
    .whenReleased(new HorizontalConveyorDoNothing(horizontalConveyerSub))
    .whenReleased(new VerticalConveyorRunWhenBall(verticalConveyerSub))
    .whenReleased(new IntakeDoNothing(intakeSub));

    //-----SPIT BALLS-----
    new JoystickButton(RobotIO.operatorStick, RobotSettings.SPIT_BUTTON)
    .whenPressed(new PIDShooter(shooterSub, 8000))
    .whenPressed(new FeedWhenShooterReady(verticalConveyerSub, 8000, 1000))
    .whenReleased(new ShooterWheelsDoNothing(shooterSub))
    .whenReleased(new VerticalConveyorRunWhenBall(verticalConveyerSub));

    //-----VERTICAL CONVEYOR UP-----
    new JoystickButton(RobotIO.operatorStick, RobotSettings.VERT_CONVEYOR_UP)
    .whenPressed(new VerticalConveyorSpin(verticalConveyerSub, 0.5))
    .whenReleased(new VerticalConveyorRunWhenBall(verticalConveyerSub));

    //-----VERTICAL CONVEYOR DOWN-----
    new JoystickButton(RobotIO.operatorStick, RobotSettings.VERT_CONVEYOR_DOWN)
    .whenPressed(new VerticalConveyorSpin(verticalConveyerSub, -0.5))
    .whenReleased(new VerticalConveyorRunWhenBall(verticalConveyerSub));
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

      return ramseteCommand;//.andThen(() -> DriveSub.tankDrive(0, 0));
  }
}