package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.CANcoder;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import edu.wpi.first.math.util.Units;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class SwerveModule {

    private SparkMax driveMotor;
    private RelativeEncoder driveEncoder;
    private SparkClosedLoopController driveController;
    private SparkMaxConfig driveConfig;

    private SparkMax turnMotor;
    private RelativeEncoder turnEncoder;
    private SparkClosedLoopController turnController;
    private SparkMaxConfig turnConfig;

    private CANcoder canCoder;
    
    public SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID)
    {
        driveMotor = new SparkMax(driveMotorCANID, SparkLowLevel.MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();   
        driveConfig = new SparkMaxConfig();
        driveConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(0)
            .voltageCompensation(0);
        driveConfig.encoder
            .positionConversionFactor(0)
            .velocityConversionFactor(0);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0, 0, 0);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



        turnMotor = new SparkMax(steerMotorCANID, SparkLowLevel.MotorType.kBrushless);
        turnEncoder = turnMotor.getEncoder();
        turnController = turnMotor.getClosedLoopController();
        turnConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(0)
            .voltageCompensation(0);
        turnConfig.encoder
            .positionConversionFactor(0)
            .velocityConversionFactor(0);
        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0, 0, 0);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Reset everything to factory default
        /*
        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();
        */

        // CANcoder Configuration
        canCoder = new CANcoder(cancoderCANID);
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor = new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(1);
        canCoder.getConfigurator().apply(canCoderConfig);
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
