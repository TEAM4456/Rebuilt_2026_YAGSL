// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;

public class Shooter extends SubsystemBase{

  private SparkMax shootLeftMotor; // Represents
  private RelativeEncoder shootLeftEncoder;
  private SparkClosedLoopController shootLeftLoop;
  private SparkMaxConfig shootLeftConfig;

  private SparkMax shootRightMotor;
  private RelativeEncoder shootRightEncoder;
  private SparkClosedLoopController shootRightLoop;
  private SparkMaxConfig shootRightConfig;

  public Shooter() {

    shootLeftMotor = new SparkMax(19, MotorType.kBrushless);
    shootLeftEncoder = shootLeftMotor.getEncoder();
    shootLeftLoop = shootLeftMotor.getClosedLoopController();

    shootLeftConfig = new SparkMaxConfig();
    shootLeftConfig.idleMode(IdleMode.kCoast);
    shootLeftConfig.closedLoop.pid(1, 0, 0);
    shootLeftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    shootLeftConfig.openLoopRampRate(0.5);
    shootLeftConfig.smartCurrentLimit(40);
    shootLeftMotor.configure(shootLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


    shootRightMotor = new SparkMax(18, MotorType.kBrushless);
    shootRightEncoder = shootRightMotor.getEncoder();
    shootRightLoop = shootRightMotor.getClosedLoopController();

    shootRightConfig = new SparkMaxConfig();
    shootRightConfig.idleMode(IdleMode.kCoast);
    shootRightConfig.closedLoop.pid(1, 0, 0);
    shootRightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    shootRightConfig.openLoopRampRate(0.5);
    shootRightConfig.smartCurrentLimit(40);
    shootRightMotor.configure(shootRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Starts both shooters spinning in shoot direction @return this command */
  public Command shooterShoot() {
    return run(
      () -> {

        // Set these to the same speed
        shootLeftMotor.set(0.8);
        shootRightMotor.set(-0.8);
      });
  }

  /** Stops both shooters and index motor basically shutting down the shooter @return this command */
  public Command shooterStop() {
    return run(
      () -> {

        // Set these to the same speed
        shootLeftMotor.set(0);
        shootRightMotor.set(0);
      });
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Right Shooter Speed", shootRightEncoder.getVelocity());
    SmartDashboard.putNumber("Right Motor Position", shootRightEncoder.getPosition());

    SmartDashboard.putNumber("Left Shooter Speed", shootLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Left Motor Position", shootLeftEncoder.getPosition());
  }
}