// Import relevant classes.
package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

// Example SwerveDrive class
public class SwerveDrive extends SubsystemBase
{

    // Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    AHRS gyro; // Psuedo-class representing a gyroscope.
    SwerveModule[] swerveModules; // Psuedo-class representing swerve modules.

    SwerveDrivePoseEstimator poseEstimator;
    public RobotConfig config;

    // Constructor
    public SwerveDrive() {
    
        swerveModules = new SwerveModule[4]; // Create swerve modules here.

        swerveModules[3] = new SwerveModule(7, 8, 9, false, 3); // Back right
        swerveModules[2] = new SwerveModule(10, 11, 12, false, 2); // Back left
        swerveModules[1] = new SwerveModule(4, 5, 6, false, 1); // Front right
        swerveModules[0] = new SwerveModule(1, 2, 3, false, 0); // Front left
        
        // Create SwerveDriveKinematics object
        // 10.5in from center of robot to center of wheel.
        // 10.5in is converted to meters to work with object.
        // Translation2d(x,y) == Translation2d(front, left)
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(-10.5), Units.inchesToMeters(10.5)), // Front Left
            new Translation2d(Units.inchesToMeters(10.5), Units.inchesToMeters(10.5)), // Front Right
            new Translation2d(Units.inchesToMeters(-10.5), Units.inchesToMeters(-10.5)), // Back Left
            new Translation2d(Units.inchesToMeters(10.5), Units.inchesToMeters(-10.5))  // Back Right
        );
        
        gyro = new AHRS(NavXComType.kMXP_SPI); // Psuedo-constructor for generating gyroscope.

        // Create the SwerveDriveOdometry given the current angle, the robot is at x=0, r=0, and heading=0
        odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(gyro.getAngle()), // returns current gyro reading as a Rotation2d
            new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
            // Front-Left, Front-Right, Back-Left, Back-Right
            new Pose2d(0,0,new Rotation2d()) // x=0, y=0, heading=0
        );


        // STARTING CREATION OF AUTOBUILDER CONFIG HERE AT THE BOTTOM OF THE CONSTRUCTOR


        var stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
        var visionStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(10));

        poseEstimator = new SwerveDrivePoseEstimator(// FROM CLASS SwerveDrivePoseEstimator DAN_F
            kinematics,
            getRotation2d(),
            getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs);
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        
        // Passing values seperated by commas
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(Constants.pDriveMotor, Constants.iDriveMotor,Constants.dDriveMotor), // Translation PID constants
                    new PIDConstants(Constants.pTurnMotor, Constants.iTurnMotor,Constants.dTurnMotor) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
            );
        
    }
    
    // Old drive method being overloaded
    public void drive(Translation2d translation, double rotation) {
        
        // This method was pulled from the old SwerveDrive code and is needed for
        // teleop control in RobotContainer through TeleopSwerve command
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getRotation2d()));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 3.5); // Used to be a constant
        
        // A for each loop that sets the desired state of each swerve module on the robot
        /*for (SwerveModule tempMod : swerveModules) {
            tempMod.setDesiredState(swerveModuleStates[tempMod.moduleNumber]);
        }
        */

        // A normal for loop that apples teh swerveModuleStates to each module
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    // Simple drive function also being overloaded. provided by YAGSL? Not used currently in our code but nice to have around
    public void drive()
    {
        // Create test ChassisSpeeds going X = 14in, Y=4in, and spins at 30deg per second.
        ChassisSpeeds testSpeeds = new ChassisSpeeds(Units.inchesToMeters(14), Units.inchesToMeters(4), Units.degreesToRadians(30));
        
        // Get the SwerveModuleStates for each module given the desired speeds.
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(testSpeeds);
        // Output order is Front-Left, Front-Right, Back-Left, Back-Right
        
        swerveModules[0].setState(swerveModuleStates[0]);
        swerveModules[1].setState(swerveModuleStates[1]);
        swerveModules[2].setState(swerveModuleStates[2]);
        swerveModules[3].setState(swerveModuleStates[3]);
    }


    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
     }
    public void resetPose(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(getStates());
        return chassisSpeeds;
     }
    public void driveRobotRelative(ChassisSpeeds speeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);
     }
    public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(targetStates[mod.moduleNumber]);
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveModules) {
        positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void updateOdometry() {
        poseEstimator.update(
        gyro.getRotation2d(),
        getModulePositions());

        boolean useMegaTag2 = true; //set to false to use MegaTag1
        boolean doRejectUpdate = false;
        if(useMegaTag2 == false)
        {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        
        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
            if(mt1.rawFiducials[0].ambiguity > .7)
            {
            doRejectUpdate = true;
            }
            if(mt1.rawFiducials[0].distToCamera > 3)
            {
            doRejectUpdate = true;
            }
        }
        if(mt1.tagCount == 0)
        {
            doRejectUpdate = true;
        }

        if(!doRejectUpdate)
        {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            poseEstimator.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds);
        }
        }
        else if (useMegaTag2 == true)
        {
        LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if(Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }
        }
    }
    
    @Override
    public void periodic()
    {
        // Update the odometry every run.
        odometry.update(Rotation2d.fromDegrees(gyro.getAngle()),  getModulePositions());
    }
    
}