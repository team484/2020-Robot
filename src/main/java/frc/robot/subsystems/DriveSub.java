/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;
import frc.robot.commands.JoystickDrive;



public class DriveSub extends SubsystemBase {
  /**
   * Creates a new DriveSub.
   */
  public DriveSub() {
    setDefaultCommand(new JoystickDrive(this));
    RobotIO.rightMotor2.follow(RobotIO.rightMotor1);
    RobotIO.rightMotor3.follow(RobotIO.rightMotor1);
    RobotIO.leftMotor2.follow(RobotIO.leftMotor1);
    RobotIO.leftMotor3.follow(RobotIO.leftMotor1);
}
   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
public static void set(double speed, double rot)
 {
RobotIO.difDrive.arcadeDrive(speed, rot);
}

public static double getLeftDistance(){
  return RobotIO.leftEncoder.getDistance();
}
public static double getRightDistance(){
  return RobotIO.rightEncoder.getDistance();
}
public static double getDistance(){
  return (getRightDistance() + getLeftDistance()) / 2;
}
}
