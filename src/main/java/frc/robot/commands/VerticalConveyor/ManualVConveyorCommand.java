package frc.robot.commands.VerticalConveyor;

//subsystems and commands
import frc.robot.subsystems.VerticalConveyorSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
//general imports
import frc.robot.Constants.JoystickConstants;

public class ManualVConveyorCommand extends CommandBase {
    //declaring subsystems
    private VerticalConveyorSubsystem m_VerticalConveyorSubsystem;

    //declaring the joystick used
    private GenericHID joystick;

    //constructor
    public ManualVConveyorCommand(GenericHID joystick, VerticalConveyorSubsystem verticalConveyorSubsystem) {
        this.joystick = joystick;
        m_VerticalConveyorSubsystem = verticalConveyorSubsystem;

        addRequirements(m_VerticalConveyorSubsystem);
    }

    @Override
    public void initialize() {
   
    }
    
    @Override
    public void execute() {
        int speed = joystick.getRawButton(JoystickConstants.kLeftBumperPort) ? 1 :  (joystick.getRawAxis(JoystickConstants.kLeftTriggerPort) > 0.1) ? -1 : 0;
        m_VerticalConveyorSubsystem.setVerticalIntakeSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_VerticalConveyorSubsystem.setVerticalIntakeSpeed(0);
    }

    //in progress
    @Override
    public boolean isFinished() {
        return joystick.getRawButton(JoystickConstants.kLeftBumperPort) && (joystick.getRawAxis(JoystickConstants.kLeftTriggerPort) <= JoystickConstants.kDeadzone) == false;
    }
}