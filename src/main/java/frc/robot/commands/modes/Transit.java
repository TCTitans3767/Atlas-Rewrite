package frc.robot.commands.modes;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.RobotTransitions;
import frc.robot.utils.State;

@State
public class Transit extends Command{

    
    public Transit() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {

        if (TriggerBoard.isClimbButtonBoxButtonPressed()) {
            if (TriggerBoard.isClimbControllerButtonPressed()) {
                RobotControl.setCurrentMode(RobotTransitions.climbPose);
            }
            return;
        }

        if (Robot.intake.isWheelMotorTooHot()) {
            Robot.intake.resetWheelSpeed();
        }

        if (!TriggerBoard.isAlgaeInIntake()) {
            Robot.intake.resetWheelSpeed();
        }

        if (TriggerBoard.isCoralButtonPressed()) {

            if (TriggerBoard.isCoralInManipulator()) {

                if (TriggerBoard.isL1Selected()) {
                    RobotControl.setCurrentMode(RobotTransitions.coralReefPose);
                    return;
                } else {
                    RobotControl.setCurrentMode(RobotTransitions.coralReefAlignPose);
                    return;
                }

            } else if (!TriggerBoard.isCoralInManipulator()) {
                RobotControl.setCurrentMode(RobotTransitions.coralStationPose);
                return;
            }

        } else if (TriggerBoard.isCoralOverrideButtonPressed()) {

            if (!TriggerBoard.isCoralInManipulator()) {
                RobotControl.setCurrentMode(RobotTransitions.coralFloorPose);
                return;
            } else if (TriggerBoard.isCoralInManipulator()) {
                RobotControl.setCurrentMode(RobotTransitions.transitPose);
                return;
            }

        } else if (TriggerBoard.isAlgaeButtonPressed()) {

            if (!TriggerBoard.isAlgaeInIntake()) {
                RobotControl.setCurrentMode(RobotTransitions.algaePickupPose);
                return;
            } else if (TriggerBoard.isAlgaeInIntake()) {
                RobotControl.setCurrentMode(RobotTransitions.ejectAlgaePose);
                return;
            }
            
        }

        if (Robot.drivetrain.distanceTo(Robot.getAlliance() == Alliance.Blue ? Constants.Field.blueReefCenter : Constants.Field.redReefCenter) >= 1.3 && Robot.joystick.a().getAsBoolean()) {
            RobotControl.setCurrentMode(RobotTransitions.knockOffAlgaePoseManual);
        }   

    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
