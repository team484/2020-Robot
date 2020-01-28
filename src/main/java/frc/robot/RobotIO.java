/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;



/**
 * Add your docs here.
 */
public class RobotIO {
    public static final WPI_TalonFX rightMotor1 = new WPI_TalonFX(RobotSettings.RIGHT_MOTOR_1_ID);
    public static final WPI_TalonFX rightMotor2 = new WPI_TalonFX(RobotSettings.RIGHT_MOTOR_2_ID);
    public static final WPI_TalonFX rightMotor3 = new WPI_TalonFX(RobotSettings.RIGHT_MOTOR_3_ID);
    public static final WPI_TalonFX leftMotor1 = new WPI_TalonFX(RobotSettings.LEFT_MOTOR_1_ID);
    public static final WPI_TalonFX leftMotor2 = new WPI_TalonFX(RobotSettings.LEFT_MOTOR_2_ID);
    public static final WPI_TalonFX leftMotor3 = new WPI_TalonFX(RobotSettings.LEFT_MOTOR_3_ID);
    public static final DifferentialDrive difDrive = new DifferentialDrive(leftMotor1, rightMotor1);
}
