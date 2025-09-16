package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetElevatorPosition extends Command{

    private double positionMeters = 0;

    public SetElevatorPosition(double positionMeters) {
        this.positionMeters = positionMeters;
        addRequirements(Robot.elevator);
    }

    @Override
    public void initialize() {
        Robot.elevator.setPosition(positionMeters);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Robot.elevator.isAtPosition();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    
}
