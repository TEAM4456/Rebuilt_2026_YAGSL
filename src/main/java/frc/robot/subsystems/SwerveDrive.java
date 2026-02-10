// Import relevant classes.
package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

// Example SwerveDrive class
public class SwerveDrive extends SubsystemBase
{

    // Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry   odometry;
    AHRS             gyro; // Psuedo-class representing a gyroscope.
    SwerveModule[]        swerveModules; // Psuedo-class representing swerve modules.
    
    // Constructor
    public SwerveDrive() 
    {
    
        swerveModules = new SwerveModule[4]; // Psuedo-code; Create swerve modules here.

        swerveModules[0] = new SwerveModule(7, 8, 9);
        swerveModules[1] = new SwerveModule(10, 11, 12);
        swerveModules[2] = new SwerveModule(4, 5, 6);
        swerveModules[3] = new SwerveModule(1, 2, 3);
        
        // Create SwerveDriveKinematics object
        // 12.5in from center of robot to center of wheel.
        // 12.5in is converted to meters to work with object.
        // Translation2d(x,y) == Translation2d(front, left)
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(10.5), Units.inchesToMeters(10.5)), // Front Left
            new Translation2d(Units.inchesToMeters(10.5), Units.inchesToMeters(-10.5)), // Front Right
            new Translation2d(Units.inchesToMeters(-10.5), Units.inchesToMeters(10.5)), // Back Left
            new Translation2d(Units.inchesToMeters(-10.5), Units.inchesToMeters(-10.5))  // Back Right
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
    }
    
    // Old drive method being overloaded
    public void drive(Translation2d translation, double rotation, /* boolean fieldRelative, */ boolean isOpenLoop) {

        // Assignment statment taken from constants and plopped in here
        final SwerveDriveKinematics swerveKinematics =

        //XY plane is robot relative with +x is forward (front of robot) and +y is left
        new SwerveDriveKinematics(
            new Translation2d(-Units.inchesToMeters(27) / 2.0, -Units.inchesToMeters(20.5) / 2.0), // Mod 0
            new Translation2d(-Units.inchesToMeters(27) / 2.0, Units.inchesToMeters(20.5) / 2.0), // Mod 1
            new Translation2d(Units.inchesToMeters(27) / 2.0, -Units.inchesToMeters(20.5) / 2.0), // Mod 2
            new Translation2d(Units.inchesToMeters(27) / 2.0, Units.inchesToMeters(20.5) / 2.0)); // Mod 3
        
        // This method was pulled from the old SwerveDrive code and is needed for
        // teleop control in RobotContainer through TeleopSwerve command
        SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getRotation2d()));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 3.5); // Used to be a constant
        
        // A for each loop that sets the desired state of each swerve module on the robot
        for (SwerveModule tempMod : swerveModules) {
            tempMod.setDesiredState(swerveModuleStates[tempMod.moduleNumber], isOpenLoop);
        }
    }

    // Simple drive function also being overloaded. provided by YAGSL?
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
    
    // Fetch the current swerve module positions.
    public SwerveModulePosition[] getCurrentSwerveModulePositions()
    {
        return new SwerveModulePosition[]{
            new SwerveModulePosition(swerveModules[0].getDistance(), swerveModules[0].getAngle()), // Front-Left
            new SwerveModulePosition(swerveModules[1].getDistance(), swerveModules[1].getAngle()), // Front-Right
            new SwerveModulePosition(swerveModules[2].getDistance(), swerveModules[2].getAngle()), // Back-Left
            new SwerveModulePosition(swerveModules[3].getDistance(), swerveModules[3].getAngle())  // Back-Right
        };
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }
    
    @Override
    public void periodic()
    {
        // Update the odometry every run.
        odometry.update(Rotation2d.fromDegrees(gyro.getAngle()),  getCurrentSwerveModulePositions());
    }
    
}