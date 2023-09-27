package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveDriveSubsystem extends SubsystemBase {

    private SwerveModule leftFrontModule;
    private SwerveModule rightFrontModule;
    private SwerveModule leftBackModule;
    private SwerveModule rightBackModule;

    private AHRS navX;

    private SwerveDriveOdometry odometry;

    public SwerveDriveSubsystem() {
        // Front Left Module Initializing
        leftFrontModule = new SwerveModule(
            SwerveModuleConstants.kLeftFrontWheelPort, 
            SwerveModuleConstants.kLeftFrontRotationPort, 
            SwerveModuleConstants.kLeftFrontDriveReversed, 
            SwerveModuleConstants.kLeftFrontRotationReversed, 
            SwerveModuleConstants.kLeftFrontCANCoderPort, 
            SwerveModuleConstants.kLeftFrontCANCoderOffset, 
            SwerveModuleConstants.kLeftFrontCANCoderReversed
        );

        // Front Right Module Initializing
        rightFrontModule = new SwerveModule(
            SwerveModuleConstants.kRightFrontWheelPort, 
            SwerveModuleConstants.kRightFrontRotationPort, 
            SwerveModuleConstants.kRightFrontDriveReversed, 
            SwerveModuleConstants.kRightFrontRotationReversed, 
            SwerveModuleConstants.kRightFrontCANCoderPort, 
            SwerveModuleConstants.kRightFrontCANCoderOffset, 
            SwerveModuleConstants.kRightFrontCANCoderReversed
        );

        leftBackModule = new SwerveModule(
            SwerveModuleConstants.kLeftBackWheelPort, 
            SwerveModuleConstants.kLeftBackRotationPort, 
            SwerveModuleConstants.kLeftBackDriveReversed, 
            SwerveModuleConstants.kLeftBackRotationReversed, 
            SwerveModuleConstants.kLeftBackCANCoderPort, 
            SwerveModuleConstants.kLeftBackCANCoderOffset, 
            SwerveModuleConstants.kLeftBackCANCoderReversed
        );

        rightBackModule = new SwerveModule(
            SwerveModuleConstants.kRightBackWheelPort, 
            SwerveModuleConstants.kRightBackRotationPort, 
            SwerveModuleConstants.kRightBackDriveReversed, 
            SwerveModuleConstants.kRightBackRotationReversed, 
            SwerveModuleConstants.kRightBackCANCoderPort, 
            SwerveModuleConstants.kRightBackCANCoderOffset, 
            SwerveModuleConstants.kRightBackCANCoderReversed
        );

        navX = new AHRS(SPI.Port.kMXP);
        navX.enableLogging(true);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                navX.calibrate();
                navX.reset();
            } 
            catch(Exception e) {

            }
        }).start();

        odometry = new SwerveDriveOdometry(
            SwerveModuleConstants.kinematics, 
            getRotation2dContinuous(),
            getModulePosition()
        );
    }

    //=========================================================================== 
    // gyro and accelorometer methods
    //===========================================================================

    //calibrates the gyro
    public void calibrateGyro() {
        navX.calibrate();
    }

    /**
     * Resets the current angle of the gyro to 0. 
     * Tells the driver that the gyro is connected via a print statement
    */
    public void zeroOutGyro() {
        System.out.println("Gyro Connected: "+navX.isConnected());
        navX.reset();
    }

    /**
     * Gets the current yaw angle from the navx gyro
    */
    public double getYaw() {
        return navX.getYaw();
    }

    /**
     * Gets the current pitch angle from the navx gyro
    */
    public double getPitch() {
        return navX.getPitch();
    }

    /**
     * Gets the current roll angle from the navx gyro
    */
    public double getRoll() {
        return navX.getRoll();
    }

    /**
     * Gets the rotation of the robot from a top down perspective
     */
    public Rotation2d getRotation2dContinuous() {
        System.out.println(getDegrees());
        return Rotation2d.fromDegrees(getDegrees());
    }

    /**
     * Gets the rotation of the robot from a top down perspective
     */
    public Rotation2d getRotation2dCWP() {
        return Rotation2d.fromDegrees(getDegreesCWP());
    }

    public Rotation2d getRotation2dRad() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public double getDegrees() {
        double rawDegrees = -getYaw();
        return rawDegrees < 0 ? rawDegrees + 360 : rawDegrees;
    }

    public double getRad() {
        double rad = Units.degreesToRadians(-getYaw()) % (2 * Math.PI);
        return rad < 0 ? rad + 2 * Math.PI : rad;
    }

    public double getDegreesCWP() {
        double rawDegrees = getYaw() % 360;
        return rawDegrees < 0 ? rawDegrees + 360 : rawDegrees;
    }

    public void shutdown() {
        leftFrontModule.shutdown();
        rightFrontModule.shutdown();
        leftBackModule.shutdown();
        rightBackModule.shutdown();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kDriveMaxMetersPerSecond);
        leftFrontModule.setSwerveState(desiredStates[0]);
        rightFrontModule.setSwerveState(desiredStates[1]);
        leftBackModule.setSwerveState(desiredStates[2]);
        rightBackModule.setSwerveState(desiredStates[3]);
    }

    public void getCANCoderReading() {
        System.out.println("Left Front: "+leftFrontModule.getCANCoderReading());
        System.out.println("Left Back: "+leftBackModule.getCANCoderReading());
        System.out.println("Right Front: "+rightFrontModule.getCANCoderReading());
        System.out.println("Right Back: "+rightBackModule.getCANCoderReading());
    }

    /**
     * Returns the current position of the module (displacement)
    * @return The current displacement of the module
    */
    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(
                leftFrontModule.getDrivePosition(), new Rotation2d(leftFrontModule.getRotationPosition())
            ),
            new SwerveModulePosition(
                rightFrontModule.getDrivePosition(), new Rotation2d(rightFrontModule.getRotationPosition())
            ),
            new SwerveModulePosition(
                leftBackModule.getDrivePosition(), new Rotation2d(leftBackModule.getRotationPosition())
            ),
            new SwerveModulePosition(
                rightBackModule.getDrivePosition(), new Rotation2d(rightBackModule.getRotationPosition())
            )
        };
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2dContinuous(), getModulePosition(), pose);
    }

    @Override
    public void periodic() {
        odometry.update(
            getRotation2dContinuous(),
            getModulePosition()
        );

        SmartDashboard.putNumber("Robot Heading", getDegrees());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }
}
