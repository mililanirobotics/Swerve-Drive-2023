package frc.robot.commands.Flywheel;

//subsystems and commands
import frc.robot.subsystems.FlywheelSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
//general imports
import frc.robot.Constants.JoystickConstants;

public class ManualFlywheelCommand extends CommandBase {
    //declaring subsystems
    private FlywheelSubsystem m_FlywheelSubsystem;

    //declaring the joystick used
    private GenericHID joystick;

    //constructor
    public ManualFlywheelCommand(GenericHID joystick, FlywheelSubsystem flywheelSubsystem) {
        this.joystick = joystick;
        m_FlywheelSubsystem = flywheelSubsystem;

        addRequirements(m_FlywheelSubsystem);
    }

    @Override
    public void initialize() {
   
    }
    
    @Override
    public void execute() {
        double speed = joystick.getRawAxis(JoystickConstants.kLeftYJoystickPort) * 0.3;
        m_FlywheelSubsystem.setFlywheelSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    //in progress
    @Override
    public boolean isFinished() {
        return Math.abs(joystick.getRawAxis(JoystickConstants.kLeftYJoystickPort)) < 0.2;
    }
}