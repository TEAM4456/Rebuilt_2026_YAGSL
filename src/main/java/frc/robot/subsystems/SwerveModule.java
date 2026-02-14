package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.CANcoder;
import java.io.ObjectInputFilter.Config;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.thethriftybot.server.CAN;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SwerveModule {

    private SparkMax driveMotor; // Represents drive motor object
    private RelativeEncoder driveEncoder; // Represents internal encoder of the drive motor
    private SparkClosedLoopController driveController; // Not sure what this represents and its never called in the code
    private SparkMaxConfig driveConfig; // Object that contains configuration info for the drive motor

    private SparkMax turnMotor; // Represents turn motor object
    private RelativeEncoder turnEncoder; // Represents internal encoder of the turn motor
    private SparkClosedLoopController turnController; // Also not sure what this represents and its never called in the code
    private SparkMaxConfig turnConfig; // Object that contains configuration info for the turn motor

    private CANcoder canCoder; // Represents the CAN coder object
    private CANcoderConfiguration canCoderConfig;
    private Rotation2d lastAngle; // Not sure what this represents, old code that probably conflicts with other stuff
    
    public SwerveModule(int driveMotorCANID, int turnMotorCANID, int canCoderCANID, boolean driveInverted) {

        // Instantiations for drive motor stuff
        driveMotor = new SparkMax(driveMotorCANID, SparkLowLevel.MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();   
        driveConfig = new SparkMaxConfig();
        // Uses method chaining to set config info for drive motor and applies it at the end
        // Everything here with type Constants is taken from our "Constants.java" file
        driveConfig
            .inverted(driveInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.driveSmartCurrentLimit)
            .voltageCompensation(Constants.driveVoltageCompensation);
        driveConfig.encoder
            .positionConversionFactor(Constants.drivePotentialConversionFactor)
            .velocityConversionFactor(Constants.driveVelocityConversionFactor);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.pDriveMotor, Constants.iDriveMotor, Constants.dDriveMotor);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // Aplies config to drive motor


        // Same proccess above is used for turn motor instantiations and config
        turnMotor = new SparkMax(turnMotorCANID, SparkLowLevel.MotorType.kBrushless);
        turnEncoder = turnMotor.getEncoder();
        turnController = turnMotor.getClosedLoopController();
        turnConfig = new SparkMaxConfig();
        turnConfig
            .inverted(Constants.turnInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.turnSmartCurrentLimit)
            .voltageCompensation(Constants.turnVoltageCompensation);
        turnConfig.encoder
            .positionConversionFactor(Constants.turnPotentialConversionFactor);
            //.velocityConversionFactor(Constants.turnVelocityConversionFactor); // Not used in old code so commented out for now
        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.pTurnMotor, Constants.iTurnMotor, Constants.dTurnMotor);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Instatiations for canCoder stuff
        canCoder = new CANcoder(canCoderCANID);
        canCoderConfig = new CANcoderConfiguration();
        // Gets the magnetSensor property from the canCoderConfig to apply magnet sensor config using method chaining
        canCoderConfig.MagnetSensor = new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(Constants.canCoderAbsoluteSensorDiscontinuityPoint);
        canCoder.getConfigurator().apply(canCoderConfig); // Aplies config to CAN coder

        // Methods to reset everything to factory default, probably want to use these in competition
        /*
        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();
        */
            
        lastAngle = getState().angle; // Look into this more at a later date
    }

    /*  
    ================================================================================================
    A lot of helper methods declared here, some of which can definetly be condensed into one another
    ================================================================================================
    */

    /**
    Get the distance in meters.
    */
    public double getDistance()
    {
        return driveEncoder.getPosition();
    }
    
    /**
    Get the angle.
    */
    public Rotation2d getAngle()
    {
          return Rotation2d.fromDegrees(turnEncoder.getPosition());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Custom optimize command, since default WPILib optimize assumes continuous controller which
        // REV and CTRE are not
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, Constants.isOpenLoop); // Old code said this was "boolean field relative", PLEASE LOOK INTO THIS
    }

    // FIXME, Constants here need to be moved to the Constants file
    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (3.5 * 0.01)) // 3.5 was a constant called Constants.Swerve.maxSpeed!!!!!!!
                ? lastAngle
                : desiredState.angle;

        turnController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

        SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                0.467, 1.44, 0.27);

        if (isOpenLoop) {
        double percentOutput = desiredState.speedMetersPerSecond / 3.5; // 3.5 was a constant called Constants.Swerve.maxSpeed!!!!!!!
        driveMotor.set(percentOutput);
        }
        else {
        driveController.setSetpoint(
            desiredState.speedMetersPerSecond,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }
    
    /**
    Set the swerve module state.
    @param state The swerve module state to set.
    */
    public void setState(SwerveModuleState state)
    {
          turnController.setReference(state.angle.getDegrees(), ControlType.kPosition);
          driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    }

}
