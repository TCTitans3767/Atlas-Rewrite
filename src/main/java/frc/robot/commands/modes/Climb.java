package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.RobotTransitions;
import frc.robot.utils.State;

@State
public class Climb extends Command{
    
    public Climb() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {

        if (!TriggerBoard.isClimbButtonBoxButtonPressed()) {
            RobotControl.setCurrentMode(RobotTransitions.initialTransitPose);
        }

        if (TriggerBoard.isClimbControllerButtonPressed()) {
            RobotControl.setCurrentMode(RobotTransitions.deployClimberPose);
        }
    }

}
