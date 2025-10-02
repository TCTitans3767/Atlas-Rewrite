package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        // pivot inputs
        public double intakePosition = 0.0;
        public double intakeVelocityRotationsPerSec = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double intakeCurrent = 0.0;
        public double intakeSetPosition = 0.0;
        public boolean isIntakeAtSetpoint = false;
        // wheel inputs
            // right wheel
        public double intakeWheelRightVelocityRotationsPerSec = 0.0;
        public double intakeWheelRightAppliedVolts = 0.0;
        public double intakeWheelRightCurrent = 0.0;
        public boolean isIntakeWheelRightAtSetpoint = false;
        public double intakeWheelRightWheelSetVelocity = 0.0;
            // left wheel
        public double intakeWheelLeftVelocityRotationsPerSec = 0.0;
        public double intakeWheelLeftAppliedVolts = 0.0;
        public double intakeWheelLeftCurrent = 0.0;
        public boolean isIntakeWheelLeftAtSetpoint = false;
        public double intakeWheelLeftWheelSetVelocity = 0.0;

        // canRange inputs
        public boolean isGamePieceInIntake = false;

        public boolean isLeftWheelMotorTooHot = false;
        public boolean isRightWheelMotorTooHot = false;
        public double intakeError = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakePosition(double position) {}

    public default void setIntakeWheelRightVelocity(double velocity) {}

    public default void setIntakeWheelLeftVelocity(double velocity) {}

    public default void setIntakeWheelSpeed(double speed) {}

    public default void setPivotSpeed(double speed) {}

}
