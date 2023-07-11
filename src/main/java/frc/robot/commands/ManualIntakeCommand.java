package frc.robot.commands;

//subsystems and commands
import frc.robot.subsystems.HorizontalConveyorSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
//general imports
import frc.robot.Constants.JoystickConstants;

public class ManualIntakeCommand extends CommandBase {
    //declaring subsystems
    private HorizontalConveyorSubsystem m_HorizontalConveyorSubsystem;

    //declaring the joystick used
    private GenericHID joystick;

    //constructor
    public ManualIntakeCommand(GenericHID joystick, HorizontalConveyorSubsystem horizontalConveyorSubsystem) {
        this.joystick = joystick;
        m_HorizontalConveyorSubsystem = horizontalConveyorSubsystem;

        addRequirements(m_HorizontalConveyorSubsystem);
    }
    
    @Override
    public void execute() { 
        int speed = joystick.getRawButton(JoystickConstants.kRightBumperPort) ? 1 : (joystick.getRawAxis(JoystickConstants.kRightTriggerPort) > JoystickConstants.kDeadzone) ? -1 : 0;
        m_HorizontalConveyorSubsystem.setIntakeSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_HorizontalConveyorSubsystem.setIntakeSpeed(0);
    }

    //in progress
    @Override
    public boolean isFinished() {
        return joystick.getRawButton(JoystickConstants.kRightBumperPort) && (joystick.getRawAxis(JoystickConstants.kLeftTriggerPort) < JoystickConstants.kDeadzone) == false;
    }
}