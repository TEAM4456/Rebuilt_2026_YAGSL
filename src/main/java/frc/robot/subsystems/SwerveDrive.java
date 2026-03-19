package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;
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
    private boolean isRed; // Alliance color affects Field view.
    // Attributes
    SwerveDriveKinematics kinematics;
    AHRS gyro; // Psuedo-class representing a gyroscope.
    SwerveModule[] swerveModules; // Psuedo-class representing swerve modules.
    public Field2d field;

    SwerveDrivePoseEstimator poseEstimator;
    public RobotConfig config;

    // Field Initializer runs before constructor. Use only for basic stuff... like initializing fields.
    {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            isRed = alliance.get() == DriverStation.Alliance.Red;
        } else {
            isRed = false;
        }
    }

    // Constructor
    public SwerveDrive() {
        swerveModules = new SwerveModule[4]; // Create swerve modules here.

        swerveModules[0] = new SwerveModule(1, 2, 3, !isRed, 0); // Front left
        swerveModules[1] = new SwerveModule(4, 5, 6, !isRed, 1); // Front right
        swerveModules[2] = new SwerveModule(10, 11, 12, !isRed, 2); // Back left
        swerveModules[3] = new SwerveModule(7, 8, 9, !isRed, 3); // Back right
        
        // Create SwerveDriveKinematics object
        // 10.5in from center of robot to center of wheel.
        // 10.5in is converted to meters to work with object.
        // Translation2d(x,y) == Translation2d(front, left)
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(10.5), Units.inchesToMeters(10.5)), // Front Left
            new Translation2d(Units.inchesToMeters(10.5), Units.inchesToMeters(-10.5)), // Front Right
            new Translation2d(Units.inchesToMeters(-10.5), Units.inchesToMeters(10.5)), // Back Left
            new Translation2d(Units.inchesToMeters(-10.5), Units.inchesToMeters(-10.5))  // Back Right
        );
        
        gyro = new AHRS(NavXComType.kMXP_SPI);
        
        //gyro.reset();
        
        field = new Field2d();

        /**  STARTING CREATION OF AUTOBUILDER CONFIG HERE AT THE BOTTOM OF THE CONSTRUCTOR */


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
            this::resetPose, // Method to reset pose (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(Constants.pDriveMotor, Constants.iDriveMotor,Constants.dDriveMotor), // Translation PID constants
                    new PIDConstants(Constants.pTurnMotor, Constants.iTurnMotor,Constants.dTurnMotor) // Rotation PID constants
            ),
            config, // The robot configuration
            this::allianceIsRed, // This will flip the path being followed to the red side of the field.
            this // Reference to this subsystem to set requirements
            );
        
    }
    
    public void drive(Translation2d translation, double rotation) {
        
        // This method was pulled from the old SwerveDrive code and is needed for
        // teleop control in RobotContainer through TeleopSwerve command
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getRotation2d()));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 3.5); // Used to be a constant

        // A normal for loop that apples the swerveModuleStates to each module
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(-gyro.getAngle(), 360));
    }

    //TODO Jank code written by Liam that may or may not work
    public Pose2d getPose() {
        Pose2d currentPose = poseEstimator.getEstimatedPosition();

        Pose2d alteredPose;
        if (!allianceIsRed()) {

            alteredPose = new Pose2d(-currentPose.getX(), -currentPose.getY(), currentPose.getRotation());
        }
        else {

            alteredPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d(Math.toRadians(currentPose.getRotation().getDegrees() + 180)));
        }

        return alteredPose;
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(getStates());
        return chassisSpeeds;
    }

    /**
     * Checks what alliance we are by querying the DriverStation
     * @return true if Red, false otherwise
     */
    public boolean allianceIsRed() {
        return isRed;
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


    // This is passed into RobotContainer, then into Robot to be called periodically during Autonomous operations
    public void updateOdometry() {
        poseEstimator.update(
        gyro.getRotation2d(),
        getModulePositions());

        // boolean useMegaTag2 = true; //set to false to use MegaTag1
        boolean doRejectUpdate = false;
        
        LimelightHelpers.SetRobotOrientation("limelight-sjc", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0); // Figure this method out
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-sjc");
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

    //TODO This is an experimental code to center the robot at the right position for shooting, currently just default
    /*
    public void onTheFlyTest() {
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
        );
        
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
        
        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
        
        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
    }
    */
    
    @Override
    public void periodic() 
    {

        // Update the pose every run.
        //poseEstimator.update(Rotation2d.fromDegrees(gyro.getAngle()),  getModulePositions());

        field.setRobotPose(getPose());

        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("poseX", getPose().getX());
        SmartDashboard.putNumber("poseY", getPose().getY());
        SmartDashboard.putNumber("Heading", getRotation2d().getDegrees());
    }
    
}
