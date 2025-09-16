package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.robotControl.RobotControl;

public class Climb extends Command{
    
    public Climb() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {

        if (!TriggerBoard.isClimbButtonBoxButtonPressed()) {
            RobotControl.setCurrentMode(RobotControl.initialTransitPose);
        }

        if (TriggerBoard.isClimbControllerButtonPressed()) {
            RobotControl.setCurrentMode(RobotControl.deployClimberPose);
        }
    }

}
