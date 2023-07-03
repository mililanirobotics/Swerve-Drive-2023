package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.VerticalConveyorConstants;

public class VerticalConveyorSubsystem extends SubsystemBase{
    private CANSparkMax leftVerticalConveyor;
    private CANSparkMax rightVerticalConveyor;

    private MotorControllerGroup verticalConveyor;

    private DigitalInput verticalSwitch;

    public VerticalConveyorSubsystem() {
        leftVerticalConveyor = new CANSparkMax(VerticalConveyorConstants.kLeftVerticalConveyor, MotorType.kBrushless);
        rightVerticalConveyor = new CANSparkMax(VerticalConveyorConstants.kRightVerticalConveyor, MotorType.kBrushless);

        leftVerticalConveyor.setInverted(VerticalConveyorConstants.kLeftVerticalConveyorReverse);
        rightVerticalConveyor.setInverted(VerticalConveyorConstants.kRightVerticalConveyorReverse);

        verticalConveyor = new MotorControllerGroup(leftVerticalConveyor, rightVerticalConveyor);

        verticalSwitch = new DigitalInput(1);
    }

    public boolean isVerticalSwitchTrue() {
        return verticalSwitch.get();
    }

    public void setVerticalIntakeSpeed(double percentPower) {
        verticalConveyor.setVoltage(percentPower * 12);
    }
}

