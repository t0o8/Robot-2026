package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbState;

public class ClimbCommand extends Command{

    public ClimbCommand() {
        addRequirements(RobotContainer.climbSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.climbSubsystem.requestDesiredState(ClimbState.CLIMBED, 5);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.climbSubsystem.getCurrentState() == ClimbState.CLIMBED;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
