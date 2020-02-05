/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ClimberSub;
import frc.robot.subsystems.ControlPanelSpinner;
import frc.robot.subsystems.ControlPanelSpinnerSub;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.HorizontalConveyor;
import frc.robot.subsystems.HorizontalConveyorSub;
import frc.robot.subsystems.IntakeArmSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VerticalConveyerSub;


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
  
  private final HorizontalConveyorSub horizotalConveyer = new HorizontalConveyorSub();

  private final IntakeArmSub intakeArmSub = new IntakeArmSub();

  private final IntakeSub intakeSub = new IntakeSub();

  private final ShooterSub shooterSub = new ShooterSub();

  private final VerticalConveyerSub verticalConveyerSub = new VerticalConveyerSub();


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

    // An ExampleCommand will run in autonomous
  }
}