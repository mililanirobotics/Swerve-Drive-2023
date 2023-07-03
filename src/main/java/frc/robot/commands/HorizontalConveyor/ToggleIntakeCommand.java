package frc.robot.commands.HorizontalConveyor;

//subsystems and commands
import frc.robot.subsystems.HorizontalConveyorSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleIntakeCommand extends CommandBase {
    //declaring subsystems
    private HorizontalConveyorSubsystem m_HorizontalConveyorSubsystem;

    private DoubleSolenoid.Value intialIntakeState;

    //constructor
    public ToggleIntakeCommand(HorizontalConveyorSubsystem horizontalConveyorSubsystem) {
        m_HorizontalConveyorSubsystem = horizontalConveyorSubsystem;

        addRequirements(m_HorizontalConveyorSubsystem);
    }

    @Override
    public void initialize() {
        intialIntakeState = m_HorizontalConveyorSubsystem.getIntakeState();
        m_HorizontalConveyorSubsystem.toggleIntake();
    }
    
    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    //in progress
    @Override
    public boolean isFinished() {
        return m_HorizontalConveyorSubsystem.getIntakeState() != intialIntakeState;
    }
}