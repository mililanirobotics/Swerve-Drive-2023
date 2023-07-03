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
    private CANSparkMax horizontalRoller;

    private MotorControllerGroup horizontalIntake;

    private DoubleSolenoid leftSolenoid;
    private DoubleSolenoid rightSolenoid;

    private DigitalInput horizontalSwitch;

    public HorizontalConveyorSubsystem() {
        horizontalConveyor = new CANSparkMax(HorizontalConveyorConstants.kHorizontalConveyor, MotorType.kBrushless);
        horizontalRoller = new CANSparkMax(0, MotorType.kBrushless);

        horizontalConveyor.setInverted(HorizontalConveyorConstants.kHorizontalConveyorReverse);
        horizontalRoller.setInverted(HorizontalConveyorConstants.kHorizontalRollerReverse);

        horizontalIntake = new MotorControllerGroup(horizontalConveyor, horizontalRoller);

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

    public void setHorizontalIntakeSpeed(double percentPower) {
        horizontalIntake.setVoltage(percentPower * 12);
    }
}

