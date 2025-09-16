package frc.robot;

import java.nio.file.Path;
import java.util.logging.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.AnalogTriggerOutput.AnalogTriggerOutputException;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutonCommands.CoralReefAlignPoseAuton;
import frc.robot.commands.AutonCommands.CoralReefPoseAuton;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.Utils.ReefPosition;

public class Autos {

    public static CoralReefAlignPoseAuton alignWithA4 = new CoralReefAlignPoseAuton(ReefPosition.A, "4", true);
    public static CoralReefAlignPoseAuton alignWithB4 = new CoralReefAlignPoseAuton(ReefPosition.B, "4", false);
    public static CoralReefAlignPoseAuton alignWithA2 = new CoralReefAlignPoseAuton(ReefPosition.A, "2", true);
    public static CoralReefAlignPoseAuton alignWithB2 = new CoralReefAlignPoseAuton(ReefPosition.B, "2", false);
    public static CoralReefAlignPoseAuton alignWithJ4 = new CoralReefAlignPoseAuton(ReefPosition.J, "4", false);
    public static CoralReefAlignPoseAuton alignWithK4 = new CoralReefAlignPoseAuton(ReefPosition.K, "4", true);
    public static CoralReefAlignPoseAuton alignWithL4 = new CoralReefAlignPoseAuton(ReefPosition.L, "4", false);
    public static CoralReefAlignPoseAuton alignWithE4 = new CoralReefAlignPoseAuton(ReefPosition.E, "4", true);
    public static CoralReefAlignPoseAuton alignWithC4 = new CoralReefAlignPoseAuton(ReefPosition.C, "4", true);
    public static CoralReefAlignPoseAuton alignWithD4 = new CoralReefAlignPoseAuton(ReefPosition.D, "4", false);
    public static CoralReefAlignPoseAuton alignWithF4 = new  CoralReefAlignPoseAuton(ReefPosition.F, "4", false);

    public static Command J4_L4_A4_B4_Lolipops() {
        try {

            PathPlannerPath leftWall_A = PathPlannerPath.fromPathFile("LeftWall-AB");
            PathPlannerPath A_LeftLolipop = PathPlannerPath.fromPathFile("A-LeftLolipop");
            PathPlannerPath leftLolipop_B = PathPlannerPath.fromPathFile("LeftLolipop-AB");
            PathPlannerPath B_MiddleLolipop = PathPlannerPath.fromPathFile("B-MiddleLolipop");
            PathPlannerPath A_RightLolipop = PathPlannerPath.fromPathFile("A-RightLolipop");
            PathPlannerPath RightLolipop_B = PathPlannerPath.fromPathFile("RightLolipop-AB");

            PathPlannerPath flipped_LeftWall_AB = PathPlannerPath.fromPathFile("LeftWall-AB").flipPath();

            CoralReefAlignPoseAuton alignWithA4 = new CoralReefAlignPoseAuton(ReefPosition.A, "4", true);
            CoralReefAlignPoseAuton alignWithB4 = new CoralReefAlignPoseAuton(ReefPosition.B, "4", false);
            CoralReefAlignPoseAuton alignWithA2 = new CoralReefAlignPoseAuton(ReefPosition.A, "2", true);
            CoralReefAlignPoseAuton alignWithB2 = new CoralReefAlignPoseAuton(ReefPosition.B, "2", false);

            return new SequentialCommandGroup(
                new InstantCommand(() -> {Robot.drivetrain.setPose(Robot.getAlliance() == Alliance.Blue ? leftWall_A.getStartingHolonomicPose().get() : flipped_LeftWall_AB.getStartingHolonomicPose().get()); RobotControl.setCurrentMode(RobotControl.transitPose);}),
                new InstantCommand(() -> {RobotControl.setDriveModeCommand(AutoBuilder.followPath(leftWall_A));}),
                new WaitUntilCommand(RobotControl::isDriveCommandFinished).finallyDo(() -> RobotControl.setCurrentMode(alignWithA4)),
                new WaitUntilCommand(() -> RobotControl.coralFloorPose.isScheduled()).finallyDo(() -> RobotControl.setDriveModeCommand(AutoBuilder.followPath(A_LeftLolipop))),
                new WaitUntilCommand(() -> RobotControl.transitPose.isScheduled()).finallyDo(() -> {RobotControl.setDriveModeCommand(AutoBuilder.followPath(leftLolipop_B));}),
                new WaitUntilCommand(RobotControl::isDriveCommandFinished).finallyDo(() -> RobotControl.setCurrentMode(alignWithB4)),
                new WaitUntilCommand(() -> RobotControl.coralFloorPose.isScheduled()).finallyDo(() -> RobotControl.setDriveModeCommand(AutoBuilder.followPath(B_MiddleLolipop))),
                new WaitUntilCommand(() -> RobotControl.transitPose.isScheduled()).finallyDo(() -> RobotControl.setCurrentMode(alignWithA2)),
                new WaitUntilCommand(() -> RobotControl.coralFloorPose.isScheduled()).finallyDo(() -> RobotControl.setDriveModeCommand(AutoBuilder.followPath(A_RightLolipop))),
                new WaitUntilCommand(() -> RobotControl.transitPose.isScheduled()).finallyDo(() -> RobotControl.setDriveModeCommand(AutoBuilder.followPath(RightLolipop_B))),
                new WaitUntilCommand(RobotControl::isDriveCommandFinished).finallyDo(() -> RobotControl.setCurrentMode(alignWithB2))
            );
            
        } catch (Exception e) {
            return Commands.none();
        }
    }

    public static Command J4_K4_L4_CoralStation() {

        try {
            PathPlannerPath rightReefStart_J = PathPlannerPath.fromPathFile("LeftReefStart-J");
            PathPlannerPath J_CoralStation = PathPlannerPath.fromPathFile("J-CoralStation");
            PathPlannerPath CoralStation_K = PathPlannerPath.fromPathFile("CoralStation-K");
            PathPlannerPath K_CoralStation = PathPlannerPath.fromPathFile("K-CoralStation");
            PathPlannerPath CoralStation_L = PathPlannerPath.fromPathFile("CoralStation-L");
            PathPlannerPath L_CoralStation = PathPlannerPath.fromPathFile("L-CoralStation");

            PathPlannerPath flippedRightReefStart_J = PathPlannerPath.fromPathFile("LeftReefStart-J").flipPath();       

            CoralReefAlignPoseAuton alignWithJ4 = new CoralReefAlignPoseAuton(ReefPosition.J, "4", false);
            CoralReefAlignPoseAuton alignWithK4 = new CoralReefAlignPoseAuton(ReefPosition.K, "4", true);
            CoralReefAlignPoseAuton alignWithL4 = new CoralReefAlignPoseAuton(ReefPosition.L, "4", false);
            
            CoralReefPoseAuton L4Pose = new CoralReefPoseAuton("4");

            return new SequentialCommandGroup(
                new InstantCommand(() -> {Robot.drivetrain.setPose(Robot.getAlliance() == Alliance.Blue ? rightReefStart_J.getStartingHolonomicPose().get() : flippedRightReefStart_J.getStartingHolonomicPose().get()); RobotControl.setCurrentMode(RobotControl.transitPose);}),
                new InstantCommand(() -> {RobotControl.setDriveModeCommand(AutoBuilder.followPath(rightReefStart_J));}),
                new WaitUntilCommand(RobotControl::isDriveCommandFinished).finallyDo(() -> RobotControl.setCurrentMode(alignWithJ4)),
                new WaitUntilCommand(() -> RobotControl.coralFloorPose.isScheduled()).finallyDo(() -> {RobotControl.setCurrentMode(RobotControl.coralStationPose); RobotControl.setDriveModeCommand(AutoBuilder.followPath(J_CoralStation));}),
                new WaitUntilCommand(() -> RobotControl.transitPose.isScheduled()).finallyDo(() -> {RobotControl.setDriveModeCommand(AutoBuilder.followPath(CoralStation_K));}),
                new WaitUntilCommand(() -> RobotControl.isDriveCommandFinished()).finallyDo(() -> RobotControl.setCurrentMode(alignWithK4)),
                new WaitUntilCommand(() -> RobotControl.coralFloorPose.isScheduled()).finallyDo(() -> {RobotControl.setCurrentMode(RobotControl.coralStationPose); RobotControl.setDriveModeCommand(AutoBuilder.followPath(K_CoralStation));}),
                new WaitUntilCommand(() -> RobotControl.transitPose.isScheduled()).finallyDo(() -> {RobotControl.setDriveModeCommand(AutoBuilder.followPath(CoralStation_L));}),
                new WaitUntilCommand(() -> RobotControl.isDriveCommandFinished()).finallyDo(() -> RobotControl.setCurrentMode(alignWithL4))
            );
        } catch (Exception e) {
            return Commands.none();
        }
    }

    public static Command E4_C4_D4_CoralStation() {

        try {
            PathPlannerPath rightReefStart_E = PathPlannerPath.fromPathFile("RightReefStart-E");
            PathPlannerPath E_CoralStation = PathPlannerPath.fromPathFile("E-CoralStation");
            PathPlannerPath CoralStation_C = PathPlannerPath.fromPathFile("CoralStation-C");
            PathPlannerPath C_CoralStation = PathPlannerPath.fromPathFile("C-CoralStation");
            PathPlannerPath CoralStation_D = PathPlannerPath.fromPathFile("CoralStation-D");
            PathPlannerPath D_CoralStation = PathPlannerPath.fromPathFile("D-CoralStation");

            PathPlannerPath flippedRightReefStart_E = PathPlannerPath.fromPathFile("RightReefStart-E").flipPath();       

            CoralReefAlignPoseAuton alignWithE4 = new CoralReefAlignPoseAuton(ReefPosition.E, "4", true);
            CoralReefAlignPoseAuton alignWithC4 = new CoralReefAlignPoseAuton(ReefPosition.C, "4", true);
            CoralReefAlignPoseAuton alignWithD4 = new CoralReefAlignPoseAuton(ReefPosition.D, "4", false);
            
            CoralReefPoseAuton L4Pose = new CoralReefPoseAuton("4");

            return new SequentialCommandGroup(
                new InstantCommand(() -> {Robot.drivetrain.setPose(Robot.getAlliance() == Alliance.Blue ? rightReefStart_E.getStartingHolonomicPose().get() : flippedRightReefStart_E.getStartingHolonomicPose().get()); RobotControl.setCurrentMode(RobotControl.transitPose);}),
                new InstantCommand(() -> {RobotControl.setDriveModeCommand(AutoBuilder.followPath(rightReefStart_E));}),
                new WaitUntilCommand(() -> RobotControl.isDriveCommandFinished()).finallyDo(() -> RobotControl.setCurrentMode(alignWithE4)),
                new WaitUntilCommand(() -> RobotControl.coralFloorPose.isScheduled()).finallyDo(() -> {RobotControl.setCurrentMode(RobotControl.coralStationPose); RobotControl.setDriveModeCommand(AutoBuilder.followPath(E_CoralStation));}),
                new WaitUntilCommand(() -> RobotControl.transitPose.isScheduled()).finallyDo(() -> {RobotControl.setDriveModeCommand(AutoBuilder.followPath(CoralStation_C));}),
                new WaitUntilCommand(() -> RobotControl.isDriveCommandFinished()).finallyDo(() -> RobotControl.setCurrentMode(alignWithC4)),
                new WaitUntilCommand(() -> RobotControl.coralFloorPose.isScheduled()).finallyDo(() -> {RobotControl.setCurrentMode(RobotControl.coralStationPose); RobotControl.setDriveModeCommand(AutoBuilder.followPath(C_CoralStation));}),
                new WaitUntilCommand(() -> RobotControl.transitPose.isScheduled()).finallyDo(() -> {RobotControl.setDriveModeCommand(AutoBuilder.followPath(CoralStation_D));}),
                new WaitUntilCommand(() -> RobotControl.isDriveCommandFinished()).finallyDo(() -> RobotControl.setCurrentMode(alignWithD4))
            );
        } catch (Exception e) {
            return Commands.none();
        }
    }

    public static Command centerAutoH() {

        CoralReefAlignPoseAuton alignWithH4 = new CoralReefAlignPoseAuton(ReefPosition.H, "4", false);

        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotControl.setCurrentMode(RobotControl.transitPose)),
            new WaitUntilCommand(() -> RobotControl.transit.isScheduled()),
            new InstantCommand(() -> RobotControl.setCurrentMode(alignWithH4))
        );
    }

    public static Command centerAutoG() {

        CoralReefAlignPoseAuton alignWithG4 = new CoralReefAlignPoseAuton(ReefPosition.G, "4", true);

        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotControl.setCurrentMode(RobotControl.transitPose)),
            new WaitUntilCommand(() -> RobotControl.transit.isScheduled()),
            new InstantCommand(() -> RobotControl.setCurrentMode(alignWithG4))
        );
    }


    public static boolean isAlignCommandFinsihed() {
        return RobotControl.isDriveCommandFinished();
    }

}
