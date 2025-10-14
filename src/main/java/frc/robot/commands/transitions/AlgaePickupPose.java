package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.RobotStates;
import frc.robot.utils.Transition;

@Transition
public class AlgaePickupPose extends Command{
    
    public AlgaePickupPose() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void initialize() {
        if (Robot.intake.isWheelMotorTooHot()) {
            this.cancel();
            return;
        } else {
            Robot.manipulator.setSpeed(0);
            Robot.arm.setPosition(-0.1);
            Robot.intake.setWheelPower(0.5);
            Robot.intake.setIntakePosition(-0.07);
        }
    }

    @Override
    public boolean isFinished() {
        return Robot.intake.isIntakeAtSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        RobotControl.setCurrentMode(RobotStates.algaePickup);
    }

}
