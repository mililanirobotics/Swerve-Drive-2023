package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveDriveSubsystem extends SubsystemBase {

    private SwerveModule leftFrontModule;
    private SwerveModule rightFrontModule;
    private SwerveModule leftBackModule;
    private SwerveModule rightBackModule;

    private AHRS navX;

    public SwerveDriveSubsystem() {
        // Front Left Module Initializing
        leftFrontModule = new SwerveModule(
            SwerveModuleConstants.kLeftFrontWheelPort, 
            SwerveModuleConstants.kLeftFrontRotationPort, 
            SwerveModuleConstants.kLeftFrontReversed, 
            SwerveModuleConstants.kLeftFrontReversed, 
            SwerveModuleConstants.kLeftFrontAbsoluteEncoderPort, 
            SwerveModuleConstants.kAbsoluteEncoderOffset, 
            SwerveModuleConstants.kLeftFrontReversed
        );
        // Front Right Module Initializing
        rightFrontModule = new SwerveModule(
            SwerveModuleConstants.kRightBackWheelPort, 
            SwerveModuleConstants.kRightBackRotationPort, 
            SwerveModuleConstants.kRightFrontReversed, 
            SwerveModuleConstants.kRightFrontReversed, 
            SwerveModuleConstants.kRightFrontAbsoluteEncoderPort, 
            SwerveModuleConstants.kAbsoluteEncoderOffset, 
            SwerveModuleConstants.kRightFrontReversed
        );

        leftBackModule = new SwerveModule(
            SwerveModuleConstants.kLeftBackWheelPort, 
            SwerveModuleConstants.kLeftBackRotationPort, 
            SwerveModuleConstants.kLeftBackReversed, 
            SwerveModuleConstants.kLeftBackReversed, 
            SwerveModuleConstants.kLeftBackAbsoluteEncoderPort, 
            SwerveModuleConstants.kAbsoluteEncoderOffset, 
            SwerveModuleConstants.kLeftBackReversed
        );

        rightBackModule = new SwerveModule(
            SwerveModuleConstants.kRightBackWheelPort, 
            SwerveModuleConstants.kRightBackRotationPort, 
            SwerveModuleConstants.kRightBackReversed, 
            SwerveModuleConstants.kRightBackReversed, 
            SwerveModuleConstants.kRightBackAbsoluteEncoderPort, 
            SwerveModuleConstants.kAbsoluteEncoderOffset, 
            SwerveModuleConstants.kLeftFrontReversed
        );

        navX = new AHRS(SPI.Port.kMXP);
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
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
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
}
