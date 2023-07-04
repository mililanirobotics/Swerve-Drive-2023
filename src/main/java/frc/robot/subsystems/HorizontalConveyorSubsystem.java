package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.HorizontalConveyorConstants;

public class HorizontalConveyorSubsystem extends SubsystemBase{
    private CANSparkMax horizontalConveyor;
    private CANSparkMax horizontalIntake;

    private MotorControllerGroup horizontalGroup;

    private DoubleSolenoid leftSolenoid;
    private DoubleSolenoid rightSolenoid;

    private DigitalInput horizontalSwitch;

    public HorizontalConveyorSubsystem() {
        horizontalConveyor = new CANSparkMax(HorizontalConveyorConstants.kHorizontalConveyor, MotorType.kBrushless);
        horizontalIntake = new CANSparkMax(HorizontalConveyorConstants.kHorizontalIntake, MotorType.kBrushless);

        horizontalConveyor.setInverted(HorizontalConveyorConstants.kHorizontalConveyorReverse);
        horizontalIntake.setInverted(HorizontalConveyorConstants.kHorizontalIntakeReverse);

        horizontalGroup = new MotorControllerGroup(horizontalConveyor, horizontalIntake);

        leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, HorizontalConveyorConstants.kLeftSolenoidForwardChannel, 
            HorizontalConveyorConstants.kLeftSolenoidReverseChannel);
        rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, HorizontalConveyorConstants.kRightSolenoidForwardChannel, 
            HorizontalConveyorConstants.kRightSolenoidReverseChannel);
        
        horizontalSwitch = new DigitalInput(0);
    }

    public DoubleSolenoid.Value getIntakeState() {
        return leftSolenoid.get();
    }

    public void deployIntake() {
        leftSolenoid.set(Value.kForward);
        rightSolenoid.set(Value.kForward);
    }

    public void stowIntake() {
        leftSolenoid.set(Value.kReverse);
        rightSolenoid.set(Value.kReverse);
    }

    public void toggleIntake() {
        leftSolenoid.toggle();
        rightSolenoid.toggle();
    }

    public boolean isHorizontalSwitchTrue() {
        return horizontalSwitch.get();
    }

    public void setIntakeSpeed(double percentPower) {
        horizontalIntake.setVoltage(percentPower * 12);
        horizontalConveyor.setVoltage(percentPower * 12);
    }
}

