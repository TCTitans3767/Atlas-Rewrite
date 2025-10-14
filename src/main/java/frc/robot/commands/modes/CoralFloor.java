package frc.robot.commands.modes;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.commands.lights.FlashLights;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.RobotTransitions;
import frc.robot.utils.State;

@State
public class CoralFloor extends Command{
    
    public CoralFloor() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {

        if (TriggerBoard.isL1Selected()) {
            Robot.intake.setWheelSpeed(30);
        } else {
            ChassisSpeeds chassisSpeed = Robot.drivetrain.getChassisSpeeds();
            double metersPerSecond = Math.sqrt(((chassisSpeed.vxMetersPerSecond) * (chassisSpeed.vxMetersPerSecond)) + ((chassisSpeed.vyMetersPerSecond) * (chassisSpeed.vyMetersPerSecond)));
            Robot.intake.setWheelSpeed((Math.abs(metersPerSecond/ Constants.Intake.starWheelCircumference) * 3) + 20);
        }

        if (!TriggerBoard.isL1Selected() && TriggerBoard.isCoralInManipulator()) {
            new FlashLights().schedule();
            Robot.intake.setWheelSpeed(-20);
            RobotControl.setCurrentMode(RobotTransitions.transitPose);
            return;
        } else if (!TriggerBoard.isL1Selected() && TriggerBoard.isCoralOverrideButtonPressed()) {
            RobotControl.setCurrentMode(RobotTransitions.transitPose);
            return;
        }

        if (TriggerBoard.isClimbButtonBoxButtonPressed()) {
            if (TriggerBoard.isClimbControllerButtonPressed()) {
                RobotControl.setCurrentMode(RobotTransitions.climbPose);
            }
            return;
        }

        if (TriggerBoard.isAlgaeButtonPressed()) {
            RobotControl.setCurrentMode(RobotTransitions.algaePickupPose);
            return;
        }

        if (TriggerBoard.isAlgaeRemoveButtonPressed()) {
            RobotControl.setCurrentMode(RobotTransitions.knockOffAlgaePoseManual);
            return;
        }

        if (TriggerBoard.isL1Selected() && TriggerBoard.isCoralInIntake()) {
            RobotControl.setCurrentMode(RobotTransitions.l1Pose);
            Robot.intake.setWheelSpeed(0);
            Robot.manipulator.setSpeed(0);
            return;
        }

        if(TriggerBoard.isCoralInManipulator()) {
            new FlashLights().schedule();
            Robot.intake.setWheelSpeed(-20);
            RobotControl.setCurrentMode(RobotTransitions.transitPose);
            return;
        }

    }

    @Override
    public void end(boolean interrupted) {
        Robot.manipulator.setSpeed(-0.05);
        Robot.intake.setWheelSpeed(0);
    }

}
