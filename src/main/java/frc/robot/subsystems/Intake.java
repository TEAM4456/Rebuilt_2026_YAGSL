// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {

  private SparkMax intakePivotRightMotor;
  private RelativeEncoder intakePivotRightEncoder;
  private SparkClosedLoopController intakePivotRightLoop;
  private SparkMaxConfig intakePivotRightConfig;

  private SparkMax intakePivotLeftMotor;
  private RelativeEncoder intakePivotLeftEncoder;
  private SparkClosedLoopController intakePivotLeftLoop;
  private SparkMaxConfig intakePivotLeftConfig;

  private SparkMax spinMotor;
  private RelativeEncoder spinEncoder;
  private SparkClosedLoopController spinLoop;
  private SparkMaxConfig spinConfig;

  private boolean isDown = false;

  public Intake() {
    intakePivotRightMotor = new SparkMax(21, MotorType.kBrushless);
    intakePivotRightEncoder = intakePivotRightMotor.getEncoder();
    intakePivotRightLoop = intakePivotRightMotor.getClosedLoopController();

    intakePivotRightConfig = new SparkMaxConfig();
    intakePivotRightConfig.idleMode(IdleMode.kBrake);
    intakePivotRightConfig.closedLoop.pid(1, 0, 0);
    intakePivotRightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    intakePivotRightConfig.openLoopRampRate(0.5);
    intakePivotRightConfig.smartCurrentLimit(40);
    intakePivotRightMotor.configure(intakePivotRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


    intakePivotLeftMotor = new SparkMax(22, MotorType.kBrushless);
    intakePivotLeftEncoder = intakePivotLeftMotor.getEncoder();
    intakePivotLeftLoop = intakePivotLeftMotor.getClosedLoopController();

    intakePivotLeftConfig = new SparkMaxConfig();
    intakePivotLeftConfig.idleMode(IdleMode.kBrake);
    intakePivotLeftConfig.closedLoop.pid(1, 0, 0);
    intakePivotLeftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    intakePivotLeftConfig.openLoopRampRate(0.5);
    intakePivotLeftConfig.smartCurrentLimit(40);
    intakePivotLeftMotor.configure(intakePivotLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


    spinMotor = new SparkMax(20, MotorType.kBrushless);
    spinEncoder = spinMotor.getEncoder();
    spinLoop = spinMotor.getClosedLoopController();

    spinConfig = new SparkMaxConfig();
    spinConfig.idleMode(IdleMode.kCoast);
    spinConfig.closedLoop.pid(1, 0, 0);
    spinConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    spinConfig.openLoopRampRate(0.5);
    spinConfig.smartCurrentLimit(40);
    spinMotor.configure(spinConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  
  }

  // Manual controls

  /** Raises intake mechanism off the field @return this command */
  public Command intakePivotTurnUp() {
    return run(
      () -> {
        
        intakePivotRightMotor.set(0.2);
        intakePivotLeftMotor.set(0.2);

        isDown = false;
      });
  }

  /** Brings the intake mechanism down onto the field @return this command */
  public Command intakePivotTurnDown() {
    return run(
      () -> {

        intakePivotRightMotor.set(-0.2);
        intakePivotLeftMotor.set(-0.2);

        isDown = true;
      });
  }
  
  public boolean getIsDown() {
    return isDown;
  }

  /** Accelerates intake spin motor to its max speed (Still being determined) @return this command */
  public Command spinStart() { //TODO: Determine intake speeds
    return run(
      () -> {

        spinMotor.set(0.2);
      });
  }

  /** Stops the spin motor @return this command */
  public Command spinStop() {
    return run(
      () -> {

        spinMotor.set(0);
      });
  }

  // Automatic controls

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intakePivotRightMotor", intakePivotRightEncoder.getPosition());
    SmartDashboard.putNumber("intakePivotLeftMotor", intakePivotLeftEncoder.getPosition());
  }
}
