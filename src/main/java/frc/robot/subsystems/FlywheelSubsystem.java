package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.RobotConstants;

public class FlywheelSubsystem extends SubsystemBase {
    private CANSparkMax leftFlywheel;
    private CANSparkMax rightFlywheel;

    private MotorControllerGroup flywheelShooter;

    private RelativeEncoder flywheelEncoder;
    
    public FlywheelSubsystem() {
        leftFlywheel = new CANSparkMax(FlywheelConstants.kLeftFlywheel, MotorType.kBrushless);
        rightFlywheel = new CANSparkMax(FlywheelConstants.kRightFlywheel, MotorType.kBrushless);

        leftFlywheel.setInverted(FlywheelConstants.kLeftFlywheelReverse);
        rightFlywheel.setInverted(FlywheelConstants.kRightFlywheelReverse);

        flywheelShooter = new MotorControllerGroup(leftFlywheel, rightFlywheel);

        flywheelEncoder = leftFlywheel.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, RobotConstants.kCountsPerRev);    
    }

    public double getFlywheelEncoder() {
        return flywheelEncoder.getPosition();
    }

    public void setFlywheelSpeed(double percentPower) {
        flywheelShooter.setVoltage(percentPower * 12);
    }
}
