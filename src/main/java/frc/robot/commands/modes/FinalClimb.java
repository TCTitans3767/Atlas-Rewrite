package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.utils.State;

@State
public class FinalClimb extends Command{
    
    public FinalClimb() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {
        // if (TriggerBoard.isClimbControllerButtonPressed()) {
        //     Robot.climber.setSpeed(0.6);
        // } else {
        //     Robot.climber.setSpeed(0);
        // }
        if (TriggerBoard.isClimbControllerButtonPressed()) {
            Robot.climber.setRotations(240);
        }
    }

}
