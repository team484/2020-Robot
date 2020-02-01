/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
<<<<<<< HEAD
import frc.robot.subsystems.DriveSub;
=======
<<<<<<< HEAD
import frc.robot.subsystems.DriveSub;
=======
<<<<<<< HEAD
import frc.robot.subsystems.DriveSub;
=======

import frc.robot.subsystems.DriveSub;

>>>>>>> 0a4addffd5e3ade100c95edd5382da2a207a4de6
>>>>>>> 2d147dcb1b872be69d994285c040e06c841198ed
>>>>>>> 8cbe23f854834b8919d56e6b78d019d45f1acecb
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
<<<<<<< HEAD
=======
<<<<<<< HEAD
>>>>>>> 8cbe23f854834b8919d56e6b78d019d45f1acecb
  private final DriveSub m_drivesub = new DriveSub();

=======
<<<<<<< HEAD
  private final DriveSub m_drivesub = new DriveSub();
=======
  
>>>>>>> 0a4addffd5e3ade100c95edd5382da2a207a4de6
>>>>>>> 2d147dcb1b872be69d994285c040e06c841198ed


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
<<<<<<< HEAD
    // An ExampleCommand will run in autonomous
=======
<<<<<<< HEAD

>>>>>>> 8cbe23f854834b8919d56e6b78d019d45f1acecb
  }

=======
  
<<<<<<< HEAD
}
=======
  }

>>>>>>> 0a4addffd5e3ade100c95edd5382da2a207a4de6
>>>>>>> 2d147dcb1b872be69d994285c040e06c841198ed
