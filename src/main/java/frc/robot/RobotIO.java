/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.DriveSub;



/**
 * Add your docs here.
 */
public class RobotIO {
   public static final WPI_TalonFX leftMotor1 = new WPI_TalonFX(RobotSettings.LEFT_MOTOR_1_ID);
   public static final WPI_TalonFX leftMotor2 = new WPI_TalonFX(RobotSettings.LEFT_MOTOR_2_ID);
   public static final WPI_TalonFX leftMotor3 = new WPI_TalonFX(RobotSettings.LEFT_MOTOR_3_ID);

   public static final WPI_TalonFX rightMotor1 = new WPI_TalonFX(RobotSettings.RIGHT_MOTOR_1_ID);
   public static final WPI_TalonFX rightMotor2 = new WPI_TalonFX(RobotSettings.RIGHT_MOTOR_2_ID);
   public static final WPI_TalonFX rightMotor3 = new WPI_TalonFX(RobotSettings.RIGHT_MOTOR_3_ID);

   public static final WPI_TalonSRX shooterMotor1 = new WPI_TalonSRX(RobotSettings.SHOOTER_MOTOR_1_ID);
   public static final WPI_TalonSRX shooterMotor2 = new WPI_TalonSRX(RobotSettings.SHOOTER_MOTOR_2_ID);

   public static final WPI_TalonFX ballConveyerHorizontal = new WPI_TalonFX(RobotSettings.HORIZONTAL_BALL_CONVEYOR_MOTOR_ID);
   public static final WPI_TalonFX ballConveyerVertical = new WPI_TalonFX(RobotSettings.VERTICAL_BALL_CONVEYOR_MOTOR_ID);
   
   public static final WPI_VictorSPX climberWheels = new WPI_VictorSPX(RobotSettings.CLIMBER_WHEELS_MOTOR_ID);

   public static final WPI_VictorSPX intakeAndControl = new WPI_VictorSPX(RobotSettings.INTAKE_AND_CONTROL_PANEL_MOTOR_ID);

   public static final CANSparkMax intakeArm = new CANSparkMax(RobotSettings.INTAKE_ARM_MOTOR_ID, MotorType.kBrushless);

   public static final CANSparkMax leftElevatorMotor = new CANSparkMax(RobotSettings.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
   public static final CANSparkMax rightElevatorMotor = new CANSparkMax(RobotSettings.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);

   public static final DifferentialDrive difDrive = new DifferentialDrive(leftMotor1, rightMotor1);
   public static final Joystick driveStick = new Joystick(RobotSettings.DRIVE_STICK_PORT);
   public static final Joystick operatorStick = new Joystick(RobotSettings.OP_STICK_PORT);

   public static final Servo clutchServo = new Servo(RobotSettings.CLUTCH_SERVO_PORT);
   

}
