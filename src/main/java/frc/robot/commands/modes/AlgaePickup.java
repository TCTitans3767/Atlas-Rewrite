package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.robotControl.RobotControl;

public class AlgaePickup extends Command{
    
    public AlgaePickup() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {

        if (TriggerBoard.isAlgaeInIntake()) {
            Robot.intake.setIntakePosition(0);
        }

        if (TriggerBoard.isAlgaeButtonPressed()) {
            RobotControl.setCurrentMode(RobotControl.ejectAlgaePose);
        }
    }

}
