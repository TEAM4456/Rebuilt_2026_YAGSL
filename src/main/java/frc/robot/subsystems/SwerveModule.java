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

    public int moduleNumber;
    private SparkMax driveMotor;
    private RelativeEncoder driveEncoder;
    private SparkClosedLoopController driveController;
    private SparkMaxConfig driveConfig;

    private SparkMax turnMotor;
    private RelativeEncoder turnEncoder;
    private SparkClosedLoopController turnController;
    private SparkMaxConfig turnConfig;

    private CANcoder canCoder;

    private Rotation2d lastAngle; // Old code, probably conflicts with other stuff
    
    public SwerveModule(int driveMotorCANID, int steerMotorCANID, int canCoderCANID) {
        driveMotor = new SparkMax(driveMotorCANID, SparkLowLevel.MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();   
        driveConfig = new SparkMaxConfig();
        // Everything here with type Constants is taken from our Constants.java file
        driveConfig
            .inverted(Constants.driveInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.driveSmartCurrentLimit)
            .voltageCompensation(Constants.driveVoltageCompensation);
        driveConfig.encoder
            .positionConversionFactor(Constants.drivePotentialConversionFactor)
            .velocityConversionFactor(Constants.driveVelocityConversionFactor);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.pDriveMotor, Constants.iDriveMotor, Constants.dDriveMotor);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        turnMotor = new SparkMax(steerMotorCANID, SparkLowLevel.MotorType.kBrushless);
        turnEncoder = turnMotor.getEncoder();
        turnController = turnMotor.getClosedLoopController();
        turnConfig = new SparkMaxConfig();
        // Everything here is also taken from our cConstants.java file
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

        // Reset everything to factory default
        /*
        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();
        */

        // CANcoder Configuration
        canCoder = new CANcoder(canCoderCANID);
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor = new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(Constants.canCoderAbsoluteSensorDiscontinuityPoint);
        canCoder.getConfigurator().apply(canCoderConfig);

        lastAngle = getState().angle;
    }
    public SwerveModule(int driveMotorCANID, int steerMotorCANID, int canCoderCANID, int moduleNum) {
        this(driveMotorCANID, steerMotorCANID, canCoderCANID);
        moduleNumber = moduleNum;
    }
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

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous controller which
        // REV and CTRE are not
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    // CONSTANTS ARE HERE PLS HEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEELP
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
