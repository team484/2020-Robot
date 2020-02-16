/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSub;;

/**
 * This command should be run in conjunction with the drive characteristics
 * tools provided by WPI. Set this to the auto command while running the tool.
 */
public class CharacterizeDrivetrain extends CommandBase {
  private NetworkTableEntry autoSpeedEntry, telemetryEntry, rotateEntry;
  double priorAutospeed;
  Number[] numberArray;

  /**
   * Creates a new CharacterizeDrivetrain.
   */
  public CharacterizeDrivetrain(DriveSub subsystem) {
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Running", false);

    autoSpeedEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    telemetryEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    rotateEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/rotate");

    priorAutospeed = 0;
    numberArray = new Number[10];
    DriveSub.resetLeftDistance();
    DriveSub.resetRightDistance();
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("l_encoder_pos", DriveSub.getLeftDistance());
    SmartDashboard.putNumber("l_encoder_rate", DriveSub.getLeftSpeed());
    SmartDashboard.putNumber("r_encoder_pos", -DriveSub.getRightDistance());
    SmartDashboard.putNumber("r_encoder_rate", -DriveSub.getRightSpeed());
    SmartDashboard.putBoolean("Running", true);
    double now = Timer.getFPGATimestamp();
    double leftPosition = DriveSub.getLeftDistance();
    double leftRate = DriveSub.getLeftSpeed();
    double rightPosition = -DriveSub.getRightDistance();
    double rightRate = -DriveSub.getRightSpeed();
    double battery = RobotController.getBatteryVoltage();
    double leftMotorVolts = DriveSub.getLeftMotorVoltage();
    double rightMotorVolts = DriveSub.getRightMotorVoltage();
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;
    DriveSub.tankDrive((rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed);

    // send telemetry data array back to NT
    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = Math.toRadians(DriveSub.getGyroAngle());

    telemetryEntry.setNumberArray(numberArray);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSub.tankDrive(0, 0);
    SmartDashboard.putBoolean("Running", false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
