package frc.robot.subsystems;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveDriveSubsystem {
    SwerveModule leftFrontModule = new SwerveModule(kLeftBackWheelPort, kLeftBackRotationPort, kLeftFrontReversed, kLeftFrontReversed, kLeftFrontAbsoluteEncoderPort, kAbsoluteEncoderOffset, kLeftFrontReversed);
}
