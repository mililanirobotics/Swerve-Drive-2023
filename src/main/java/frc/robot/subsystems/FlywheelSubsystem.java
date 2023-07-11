package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.FlywheelConstants;

public class FlywheelSubsystem extends SubsystemBase {
    private CANSparkMax leftFlywheel;
    private CANSparkMax rightFlywheel;

    private MotorControllerGroup flywheelShooter;
    
    public FlywheelSubsystem() {
        leftFlywheel = new CANSparkMax(FlywheelConstants.kLeftFlywheel, MotorType.kBrushless);
        rightFlywheel = new CANSparkMax(FlywheelConstants.kRightFlywheel, MotorType.kBrushless);

        leftFlywheel.setInverted(FlywheelConstants.kLeftFlywheelReverse);
        rightFlywheel.setInverted(FlywheelConstants.kRightFlywheelReverse);

        flywheelShooter = new MotorControllerGroup(leftFlywheel, rightFlywheel);
    }

    public void setFlywheelSpeed(double percentPower) {
        flywheelShooter.setVoltage(percentPower * 12);
    }
}
