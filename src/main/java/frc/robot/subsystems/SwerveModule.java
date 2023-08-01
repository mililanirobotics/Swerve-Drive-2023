package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {
    //Spark max motor controllers
    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;
    //encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;
    //PID controller
    private final PIDController rotationPID;
    //angle offsets
    private final CANCoder angleCANCoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffset;

    //constructor
    public SwerveModule(int drivePort, int rotationPort, boolean driveReversed, boolean rotationReversed,
            int absoluteEncoderPort, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        //initializing absolute encoder parameters 
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        angleCANCoder = new CANCoder(absoluteEncoderPort);
        angleCANCoder.setPositionToAbsolute();
        angleCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        angleCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        
        //initializing SparkMax
        driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationPort, MotorType.kBrushless);
        driveMotor.setInverted(driveReversed);
        rotationMotor.setInverted(rotationReversed);

        //initializing encoders
        driveEncoder = driveMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        rotationEncoder = rotationMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);   

        //converting native units to measurements
        //driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kRotationToMeters);
        // driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kMetersPerSecond);
        //rotationEncoder.setPositionConversionFactor(SwerveModuleConstants.kRotationToRadians);
        // rotationEncoder.setVelocityConversionFactor(SwerveModuleConstants.kRadiansPerSecond);

        //initializing PID controller
        rotationPID = new PIDController(
            SwerveModuleConstants.kTurningP, 
            SwerveModuleConstants.kTurningI, 
            SwerveModuleConstants.kTurningD
        );
        //calculates the least turning degrees to setpoint
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    //=========================================================================== 
    // helper methods
    //===========================================================================

    /**
     * Returns the current position of the swerve module's drive motor
     * @return The current displacement of the drive motor in meters
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Returns the current position of the swerve module's rotation motor
     * @return The current rotation of the rotation motor in radians
     */
    public double getRotationPosition() {
        return rotationEncoder.getPosition();
    }

    /**
     * Returns the current velocity of the swerve module's drive motor
     * @return The current velocity of the drive motor in meters per second
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Returns the current velocity of the swerve module's rotation motor
     * @return The current velocity of the rotation motor in radians per second
     */
    public double getRotationVelocity() {
        return rotationEncoder.getVelocity();
    }

    /**
     * Returns the current reading of the absolute encoder
     * Indicates which direction the motor should turn based on absoluteEncoderReversed 
     * @return The current reading of the absolute encoder in radians
     */
    public double getCANCoderReading() {
        return angleCANCoder.getAbsolutePosition()* (absoluteEncoderReversed ? 1 : 1);
    }

    /**
     * Resets the encoders to their default position
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(getCANCoderReading());
    }

    /**
     * Returns the current state of the swerve drive module (velocity and turn angle)
     * Optimizes the module's path to a given setpoint
     * @return Swerve module state object with the module's current position and 
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getRotationPosition()));
    }

    /**
     * Returns the current position of the module (displacement)
    * @return The current displacement of the module
    */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(rotationEncoder.getPosition()));
    }

    /**
     * Stops the movement of the drive and rotation motor 
     */
    public void shutdown() {
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    /**
     * Sets the optimal swerve module state to a given setpoint 
     * Changes the target drive and rotation speed of the module
     * @param currentState The swerve module state of the motor
     */
    public void setSwerveState(SwerveModuleState currentState) {
        if(Math.abs(currentState.speedMetersPerSecond) < 0.001) {
            shutdown();
            return;
        }

        currentState = SwerveModuleState.optimize(currentState, getModuleState().angle);
        driveMotor.set(currentState.speedMetersPerSecond / DriveConstants.kDriveMaxMetersPerSecond);
        rotationMotor.set(rotationPID.calculate(getRotationPosition(), currentState.angle.getRadians()));
    }
}

