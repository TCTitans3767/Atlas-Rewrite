package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final Alert wheelsTooHotAlert;

    public Intake(IntakeIO io) {
        this.io = io;
        wheelsTooHotAlert = new Alert("Intake Wheel Motors Too Hot! They will not run until they cool down.", Alert.AlertType.kWarning);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        wheelsTooHotAlert.set(isWheelMotorTooHot());
        if (wheelsTooHotAlert.get()) {
            resetWheelSpeed();
        }
    }

    public void setPivotSpeed(double speed) {
        io.setPivotSpeed(speed);
    }

    public void setIntakePosition(double position) {
        io.setIntakePosition(position);
    }
    public void setIntakeWheelLeftVelocity(double velocity) {
        io.setIntakeWheelLeftVelocity(velocity);
    }
    public void setIntakeWheelRightVelocity(double velocity) {
        io.setIntakeWheelRightVelocity(velocity);
    }

    public boolean isIntakeAtSetpoint() {
        return inputs.isIntakeAtSetpoint;
    }

    public boolean isIntakeWheelLeftAtSetpoint() {
        return inputs.isIntakeWheelLeftAtSetpoint;
    }

    public boolean isIntakeWheelRightAtSetpoint() {
        return inputs.isIntakeWheelRightAtSetpoint;
    }

    public boolean hasCoral() {
        return inputs.isGamePieceInIntake;
    }

    @AutoLogOutput(key = "Intake/isAlgaeInIntake")
    public boolean hasAlgae() {
        return inputs.intakeWheelLeftCurrent >= 70;
    }

    public void scoreL1() {
        setIntakeWheelLeftVelocity(-30);
        setIntakeWheelRightVelocity(-15);
    }

    public void resetWheelSpeed() {
        io.setIntakeWheelSpeed(0);
    }

    public void setWheelSpeed(double speed) {
        setIntakeWheelLeftVelocity(speed);
        setIntakeWheelRightVelocity(speed);
    }

    public void setWheelPower(double power) {
        io.setIntakeWheelSpeed(power);
    }

    public boolean isWheelMotorTooHot() {
        return inputs.isLeftWheelMotorTooHot || inputs.isRightWheelMotorTooHot;
    }

    public double getWheelSpeed() {
        return inputs.intakeWheelLeftVelocityRotationsPerSec;
    }

    public double getPivotPosition() {
        return inputs.intakePosition;
    }
}
