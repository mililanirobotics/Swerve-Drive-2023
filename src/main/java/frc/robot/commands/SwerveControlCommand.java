package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.filter.SlewRateLimiter;

import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.DriveConstants;


public class SwerveControlCommand extends CommandBase{
    private ChassisSpeeds chassisSpeeds;

    // Declaring the Subsystem
    private SwerveDriveSubsystem m_SwerveDriveSubsystem;

    // SlewRateLimiter limits the rate of acceleration to be gradual and linear
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    
    private Supplier<Double> xSpeedInput, ySpeedInput, turningSpeedInput;
    private Supplier<Boolean> fieldOrientedFunction;

    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;

    private GenericHID joystick;

    
    public SwerveControlCommand(SwerveDriveSubsystem swerveDriveSubsystem, Supplier<Double> xSpeedInput, Supplier<Double> ySpeedInput, Supplier<Double> turningSpeedInput, GenericHID joystick) {

        m_SwerveDriveSubsystem = swerveDriveSubsystem;

        this.xSpeedInput = xSpeedInput;
        this.ySpeedInput = ySpeedInput;
        this.turningSpeedInput = turningSpeedInput;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxMetersPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxMetersPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleRotationMaxRadiansPerSecond);

        this.joystick = joystick;

        addRequirements(m_SwerveDriveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Grabs Joystick Inputs as Speed Inputs
        xSpeed = xSpeedInput.get();
        ySpeed = ySpeedInput.get();
        turningSpeed = turningSpeedInput.get();
        
        System.out.println(xSpeed+", "+ySpeed);

        // Apply Deadband to prevent motors accidentally spinning
        xSpeed = Math.abs(xSpeed) > JoystickConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > JoystickConstants.kDeadband ? ySpeed : 0.0; 

        //Limiting Drive Speeds Acceleration to be linear

        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleRotationMaxRadiansPerSecond;

        // Creating desired chassis speeds from joystick inputs.
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, turningSpeed, m_SwerveDriveSubsystem.getRotation2d()
        );

        // Convert chassis speeds into swerve module states
        SwerveModuleState[] moduleStates = SwerveModuleConstants.kinematics.toSwerveModuleStates(chassisSpeeds);
        
        // Output each module state to the wheels
        m_SwerveDriveSubsystem.setModuleStates(moduleStates);

        // Temporary CANCoder print
        m_SwerveDriveSubsystem.getCANCoderReading();
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_SwerveDriveSubsystem.shutdown();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
        
}
