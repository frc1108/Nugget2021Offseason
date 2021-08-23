// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.pantherlib.DriveUtil;
import frc.robot.Constants.*;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax m_leftLead = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_leftFollow = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
  private final CANSparkMax m_rightLead = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_rightFollow = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

  private final CANEncoder m_leftEncoder;
  private final CANEncoder m_rightEncoder;
  
  private final CANPIDController m_pidLeft;
  private final CANPIDController m_pidRight;
  private final SimpleMotorFeedforward m_leftfeedforward = 
                  new SimpleMotorFeedforward(DriveConstants.ksVolts,DriveConstants.kvVoltSecondsPerMeter,
                                             DriveConstants.kaVoltSecondsSquaredPerMeter);
  private final SimpleMotorFeedforward m_rightfeedforward = 
                  new SimpleMotorFeedforward(DriveConstants.ksVolts,DriveConstants.kvVoltSecondsPerMeter,
                                             DriveConstants.kaVoltSecondsSquaredPerMeter);

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    // Stops drive motors
    stop();

    // Left side settings
    m_leftLead.restoreFactoryDefaults();
    m_leftLead.setIdleMode(IdleMode.kBrake);
    m_leftLead.setSmartCurrentLimit(40, 60);
    m_leftLead.setInverted(false);
    m_leftFollow.follow(m_leftLead);
    m_leftEncoder = m_leftLead.getEncoder();
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kDistanceMetersPerEncoderUnits);
    m_leftEncoder.setVelocityConversionFactor(DriveConstants.kVelocityMetersPerSecondPerEncoderUnits);
    m_pidLeft = m_leftLead.getPIDController();
    m_pidLeft.setP(DriveConstants.kPDriveVel);
    m_leftLead.burnFlash();

    // Right side settings
    m_rightLead.restoreFactoryDefaults();
    m_rightLead.setIdleMode(IdleMode.kBrake);
    m_rightLead.setSmartCurrentLimit(40, 60);
    m_rightLead.setInverted(true);
    m_rightFollow.follow(m_leftLead);
    m_rightEncoder = m_rightLead.getEncoder();
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kDistanceMetersPerEncoderUnits);
    m_rightEncoder.setVelocityConversionFactor(DriveConstants.kVelocityMetersPerSecondPerEncoderUnits);
    m_pidRight = m_rightLead.getPIDController();
    m_pidRight.setP(DriveConstants.kPDriveVel);
    m_rightLead.burnFlash();  
  }
  /**
   * Tank drive with Spark Max PID using WPILib feedforward
   * 
   * @param leftSpeed Drive speed setpoint for left side meters per second (max = ~3 m/s)
   * @param rightSpeed Drive speed setpoint for right side meters per second
   */
  public void tankDriveClosedLoop(double leftSpeed, double rightSpeed) {
    m_pidLeft.setReference(leftSpeed, ControlType.kVelocity, 
                           DriveConstants.kSlotDriveLeftPID, m_leftfeedforward.calculate(leftSpeed));
    m_pidRight.setReference(rightSpeed, ControlType.kVelocity, 
                            DriveConstants.kSlotDriveRightPID, m_rightfeedforward.calculate(rightSpeed));
  }

  /**
   * 
   * @param xSpeed 
   * @param zRotation
   */
  public void arcadeDriveClosedLoop(double xSpeed, double zRotation) {
    var speeds = DriveUtil.arcadeDriveIK(DriveUtil.applyDeadband(xSpeed, 0.05), DriveUtil.applyDeadband(zRotation, 0.05), true);
    tankDriveClosedLoop(DriveConstants.kMaxSpeedMetersPerSecond*speeds.left, 
                        DriveConstants.kMaxSpeedMetersPerSecond*speeds.right);
  }

   /**
   * Stops all the Drive subsytem motors
   */
  public void stop(){
    m_leftLead.stopMotor();
    m_rightLead.stopMotor();
  }
  
  
}
