package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.RobotStates;
import frc.robot.utils.Transition;

@Transition
public class CoralStationPose extends Command{

    public CoralStationPose() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void initialize() {
        if (!TriggerBoard.isAutonActive()) {
            RobotControl.setDriveModeCommand(RobotControl.controllerDrive);
        }
        Robot.intake.setIntakePosition(Constants.Intake.pivotStowPosition);
        Robot.arm.setPosition(-0.378);
        Robot.manipulator.setSpeed(-0.2);
        Robot.elevator.setPosition(0.53);
    }

    @Override
    public boolean isFinished() {
        return Robot.arm.isAtPosition() && Robot.elevator.isAtPosition();
    }

    @Override
    public void end(boolean interrupted) {
        RobotControl.setCurrentMode(RobotStates.coralStation);
    }
    
}
