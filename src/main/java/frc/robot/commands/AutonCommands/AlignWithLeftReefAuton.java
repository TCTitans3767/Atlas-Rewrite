package frc.robot.commands.AutonCommands;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ReefTagIDs;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.limelight.Camera;
import frc.robot.utils.DrivetrainPublisher;
import frc.robot.utils.Utils.ReefPosition;
import org.littletonrobotics.junction.Logger;

public class AlignWithLeftReefAuton extends Command{
    
    private final Camera camera = Robot.camera;
    private final Drivetrain drivetrain = Robot.drivetrain;

    private final RobotCentric driveWithTag = new RobotCentric();
    private final FieldCentric driveWithOdometry = new FieldCentric();
    private final Pigeon2 rotation = drivetrain.getPigeon2();

    private final PIDController xController = new PIDController(Constants.Drive.XAlignementPIDkP, Constants.Drive.XAlignementPIDkI, Constants.Drive.XAlignementPIDkD);
    private final PIDController yController = new PIDController(Constants.Drive.YAlignementPIDkP, Constants.Drive.YAlignementPIDkI, Constants.Drive.YAlignementPIDkD);
    private final PIDController headingController = new PIDController(Constants.Drive.rotationAlignementPIDkP, Constants.Drive.rotationAlignementPIDkI, Constants.Drive.rotationAlignementPIDkD);

    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;

    private final ReefPosition targetReef;

    private int targetReefTag;
    private Pose2d targetReefPose;
    private Rotation2d targetReefRotation;
    private Pose2d targetPose;
    private final Field2d field = drivetrain.getField();
    
    public AlignWithLeftReefAuton(ReefPosition targetReef) {

        this.targetReef = targetReef;

        headingController.enableContinuousInput(-180, 180);

        switch (targetReef) {
            case A:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Camera.getTagPose(ReefTagIDs.blueReefAB) : Camera.getTagPose(ReefTagIDs.redReefAB);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(0)) : new Rotation2d(Units.degreesToRadians(180));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefAB : ReefTagIDs.redReefAB;
                break;
        
            case C:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Camera.getTagPose(ReefTagIDs.blueReefCD) : Camera.getTagPose(ReefTagIDs.redReefCD);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(60)) : new Rotation2d(Units.degreesToRadians(-120));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefCD : ReefTagIDs.redReefCD;
                break;

            case E:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Camera.getTagPose(ReefTagIDs.blueReefEF) : Camera.getTagPose(ReefTagIDs.redReefEF);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(120)) : new Rotation2d(Units.degreesToRadians(-60));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefEF : ReefTagIDs.redReefEF;
                break;

            case G:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Camera.getTagPose(ReefTagIDs.blueReefGH) : Camera.getTagPose(ReefTagIDs.redReefGH);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(180)) : new Rotation2d(Units.degreesToRadians(0));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefGH : ReefTagIDs.redReefGH;
                break;

            case I:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Camera.getTagPose(ReefTagIDs.blueReefIJ) : Camera.getTagPose(ReefTagIDs.redReefIJ);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(-120)) : new Rotation2d(Units.degreesToRadians(60));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefIJ : ReefTagIDs.redReefIJ;
                break;

            case K:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Camera.getTagPose(ReefTagIDs.blueReefKL) : Camera.getTagPose(ReefTagIDs.redReefKL);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(-60)) : new Rotation2d(Units.degreesToRadians(120));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefKL : ReefTagIDs.redReefKL;
                break;

            default:
                this.cancel();
                break;
        }

        xController.setPID(Constants.Drive.XAlignementPIDkP, Constants.Drive.XAlignementPIDkI, Constants.Drive.XAlignementPIDkD);
        yController.setPID(Constants.Drive.YAlignementPIDkP, Constants.Drive.YAlignementPIDkI, Constants.Drive.YAlignementPIDkD);
        headingController.setPID(Constants.Drive.rotationAlignementPIDkP, Constants.Drive.rotationAlignementPIDkI, Constants.Drive.rotationAlignementPIDkD);

        // odometryTargetPose = targetReefPose.transformBy(new Transform2d((Constants.Robot.chassisDepthMeters/2), Units.inchesToMeters(-6), new Rotation2d(0)));
        targetPose = new Pose2d(targetReefPose.transformBy(new Transform2d((Constants.Robot.chassisDepthMeters/2), -0.24, new Rotation2d())).getTranslation(), targetReefRotation);
        // targetPose = new Pose2d(targetReefPose.getX(), targetReefPose.getY() + Units.inchesToMeters(7.5), targetReefPose.getRotation());

        Logger.recordOutput("Alignment/Target Pose", targetPose.toString());

        headingController.setTolerance(Constants.Drive.headingAlignmentTolerance);
        headingController.setSetpoint(targetPose.getRotation().getDegrees());
        headingController.reset();

        xController.setTolerance(Constants.Drive.XAlignmentTolerance);
        yController.setTolerance(Constants.Drive.YAlignmentTolerance);

    }

    @Override
    public void initialize() {
        System.out.println("entered left reef align");

        DrivetrainPublisher.setSuppliers(() -> xVelocity, () -> yVelocity, () -> rotationVelocity, () -> true);
    }

    @Override
    public void execute() {
        
        xVelocity = xController.calculate(drivetrain.getPose().getX(), targetPose.getX()) + (xController.getError() < 0 ? -Constants.Drive.XFeedForward : Constants.Drive.XFeedForward);
        yVelocity = yController.calculate(drivetrain.getPose().getY(), targetPose.getY()) + (yController.getError() < 0 ? -Constants.Drive.YFeedForward : Constants.Drive.YFeedForward);
        rotationVelocity = headingController.calculate(drivetrain.getPose().getRotation().getDegrees()) + (headingController.getError() < 0 ? Constants.Drive.rotationalFeedForward : -Constants.Drive.rotationalFeedForward);

        Logger.recordOutput("Alignment/Is Aligned", isAligned());
        
    }

    @Override
    public boolean isFinished() {
        return isAligned();
    }

    @Override
    public void end(boolean interrupted) {
        DrivetrainPublisher.setSuppliers(() -> 0, () -> 0, () -> 0);
        camera.resetTagFilter();
    }

    public boolean isAligned() {
        Logger.recordOutput("Alignment/Distance X", xController.getError());
        Logger.recordOutput("Alignment/Distance Y", yController.getError());
        return xController.atSetpoint() && yController.atSetpoint();
    }
}
