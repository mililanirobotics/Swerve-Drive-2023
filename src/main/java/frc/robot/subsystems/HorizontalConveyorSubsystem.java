package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.HorizontalConveyorConstants;

public class HorizontalConveyorSubsystem extends SubsystemBase{
    private CANSparkMax horizontalConveyor;
    private CANSparkMax horizontalIntake;

    private MotorControllerGroup horizontalGroup;

    public HorizontalConveyorSubsystem() {
        horizontalConveyor = new CANSparkMax(HorizontalConveyorConstants.kHorizontalConveyor, MotorType.kBrushless);
        horizontalIntake = new CANSparkMax(HorizontalConveyorConstants.kHorizontalIntake, MotorType.kBrushless);

        horizontalConveyor.setInverted(HorizontalConveyorConstants.kHorizontalConveyorReverse);
        horizontalIntake.setInverted(HorizontalConveyorConstants.kHorizontalIntakeReverse);

        horizontalGroup = new MotorControllerGroup(horizontalConveyor, horizontalIntake);
    }

    public void setIntakeSpeed(double percentPower) {
        horizontalGroup.setVoltage(percentPower * 12);
    }
}

