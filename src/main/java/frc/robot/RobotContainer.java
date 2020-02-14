/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//WPILib Imports
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


//Command Imports

import frc.robot.commands.IntakeSpin;
import frc.robot.commands.JoystickClimber;
import frc.robot.commands.JoystickElevator;
import frc.robot.commands.ShooterSpinWheels;

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
    autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
    autoChooser.addOption("Also Do Nothing", new WaitCommand(0));
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
      return new WaitCommand(0);
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
    new JoystickButton(RobotIO.driveStick, RobotSettings.ELEVATOR_CONTROLS_BUTTON_ID)
    .whileHeld(new JoystickElevator(elevatorSub))
    .whileHeld(new JoystickClimber(climberSub));
    
    //Operator Commands
    //-----shoot ball-----
    new JoystickButton(RobotIO.driveStick, RobotSettings.BALL_SHOOTER_BUTTON_ID)
    .whileHeld(new ShooterSpinWheels(shooterSub));
    
    //Placeholder for shooter aiming buttons (buttons 3 and 5)
    //Placeholder for the buttons that bring the intake arm to a specific height (buttons 7,9 and 11)
    new JoystickButton(RobotIO.driveStick, RobotSettings.INTAKE_WHEELS_AND_CONTROL_SPIN_BUTTON_ID).whileHeld(new IntakeSpin(intakeSub, 1.0));
    //Placeholder for automated control panel button commands
  }
}