package frc.robot.commands.HorizontalConveyor;

//subsystems and commands
import frc.robot.subsystems.HorizontalConveyorSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DeployIntakeCommand extends CommandBase {
    //declaring subsystems
    private HorizontalConveyorSubsystem m_HorizontalConveyorSubsystem;

    //constructor
    public DeployIntakeCommand(HorizontalConveyorSubsystem horizontalConveyorSubsystem) {
        m_HorizontalConveyorSubsystem = horizontalConveyorSubsystem;

        addRequirements(m_HorizontalConveyorSubsystem);
    }

    @Override
    public void initialize() {
        m_HorizontalConveyorSubsystem.deployIntake();
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
        return m_HorizontalConveyorSubsystem.getIntakeState() == Value.kForward;
    }
}