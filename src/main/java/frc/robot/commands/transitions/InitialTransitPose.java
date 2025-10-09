package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.Intake.SetIntakePosition;
import frc.robot.commands.Intake.SetIntakeWheelSpeed;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.robotControl.RobotControl;

public class InitialTransitPose extends SequentialCommandGroup{
    
    public InitialTransitPose() {

        addCommands(
            new SetArmAngle(-0.15),
            new SetIntakeWheelSpeed(0),
            new SetIntakePosition(0.32),
            new SetElevatorPosition(0.2),
            new InstantCommand(() -> System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa")),
            new SetArmAngle(-0.122),
            new InstantCommand(() -> {
                RobotControl.setDriveModeCommand(RobotControl.controllerDrive); Robot.manipulator.setSpeed(0); Robot.camera.turnOnAprilTags();}),
            new InstantCommand(() -> {RobotControl.setCurrentMode(RobotControl.transit);}).ignoringDisable(true)
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
