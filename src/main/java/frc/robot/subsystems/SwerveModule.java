package frc.robot.subsystems;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {
    //Spark max motor controllers
    private final CANSparkMax wheelMotor;
    private final CANSparkMax rotationMotor;
    //encoders
    private final RelativeEncoder wheelEncoder;
    private final RelativeEncoder rotationEncoder;
    //PID controller
    private final PIDController rotationPID;
    //angle offsets
    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffset;

    //constructor
    public SwerveModule(int wheelPort, int rotationPort, boolean wheelReversed, boolean rotationReversed,
            int absoluteEncoderPort, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        //initializing absolute encoder parameters 
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        absoluteEncoder = new AnalogInput(absoluteEncoderPort);
        
        //initializing SparkMax
        wheelMotor = new CANSparkMax(wheelPort, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationPort, MotorType.kBrushless);
        wheelMotor.setInverted(wheelReversed);
        rotationMotor.setInverted(rotationReversed);

        //initializing encoders
        wheelEncoder = wheelMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();   

        //initializing PID controller
        rotationPID = new PIDController(
            SwerveModuleConstants.kTurningP, 
            SwerveModuleConstants.kTurningI, 
            SwerveModuleConstants.kTurningD
        );
        //calculates the least turning degrees to setpoint
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

    }
    
}
