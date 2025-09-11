package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

public class IntakeIOHardware implements IntakeIO{
    private final TalonFX leftWheelMotor, rightWheelMotor, pivotMotor;
    private final TalonFXConfiguration leftWheelConfig, rightWheelConfig, pivotConfig;

    private final Slot0Configs PivotSlot0Config, leftWheelSlot0Config, rightWheelSlot0Config;
    private final MotionMagicConfigs motionMagicConfig, wheelMotionMagicConfig;

    private final CANrange intakeSensor;
    private final CANrangeConfiguration intakeSensorConfig;

    private final CANcoder pivotEncoder;

    private double targetRotations = 0;
    private double targetLeftWheelVelocity = 0;
    private double targetRightWheelVelocity = 0;

    private StatusSignal<Angle> intakePosition;
    private StatusSignal<AngularVelocity> intakeVelocity;
    private StatusSignal<Voltage> intakeAppliedVolts;
    private StatusSignal<Current> intakeCurrent;

    private StatusSignal<AngularVelocity> wheelVelocityLeft;
    private StatusSignal<AngularVelocity> wheelVelocityRight;
    private StatusSignal<Voltage> wheelAppliedVoltsLeft;
    private StatusSignal<Voltage> wheelAppliedVoltsRight;
    private StatusSignal<Current> wheelCurrentLeft;
    private StatusSignal<Current> wheelCurrentRight;

    private StatusSignal<Boolean> isGamePieceInIntake;

    public IntakeIOHardware() {
        // Motor basic setup
        leftWheelMotor = new TalonFX(Constants.Intake.leftWheelMotorID);
        leftWheelConfig = new TalonFXConfiguration();
        leftWheelConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.wheelCurrentLimit;
        leftWheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftWheelConfig.Feedback.SensorToMechanismRatio = Constants.Intake.wheelConversionFactor;
        leftWheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        rightWheelMotor = new TalonFX(Constants.Intake.rightWheelMotorID);
        rightWheelConfig = new TalonFXConfiguration();
        rightWheelConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.wheelCurrentLimit;
        rightWheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightWheelConfig.Feedback.SensorToMechanismRatio = Constants.Intake.wheelConversionFactor;
        rightWheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        pivotEncoder = new CANcoder(Constants.Intake.pivotEncoderID);

        pivotMotor = new TalonFX(Constants.Intake.pivotMotorID);
        pivotConfig = new TalonFXConfiguration();
        pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.pivotCurrentLimit;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.Feedback.FeedbackRemoteSensorID = Constants.Intake.pivotEncoderID;
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivotConfig.Feedback.RotorToSensorRatio = Constants.Intake.pivotConversionFactor;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


        // Slot 0 PID setup
        PivotSlot0Config = new Slot0Configs();
        PivotSlot0Config.kP = Constants.Intake.kP;
        PivotSlot0Config.kI = Constants.Intake.kI;
        PivotSlot0Config.kD = Constants.Intake.kD;
        PivotSlot0Config.kG = Constants.Intake.kG;
        PivotSlot0Config.kV = Constants.Intake.kV;
        PivotSlot0Config.kS = Constants.Intake.kS;
        PivotSlot0Config.GravityType = GravityTypeValue.Arm_Cosine;

        // Slot 0 PID setup
        leftWheelSlot0Config = new Slot0Configs();
        leftWheelSlot0Config.kP = Constants.Intake.wheelkP;
        leftWheelSlot0Config.kI = Constants.Intake.wheelkI;
        leftWheelSlot0Config.kD = Constants.Intake.wheelkD;
        leftWheelSlot0Config.kG = Constants.Intake.wheelkG;
        leftWheelSlot0Config.kV = Constants.Intake.wheelkV;
        leftWheelSlot0Config.kS = Constants.Intake.wheelkS;
        leftWheelSlot0Config.GravityType = GravityTypeValue.Elevator_Static;

        // Slot 0 PID setup
        rightWheelSlot0Config = new Slot0Configs();
        rightWheelSlot0Config.kP = Constants.Intake.wheelkP;
        rightWheelSlot0Config.kI = Constants.Intake.wheelkI;
        rightWheelSlot0Config.kD = Constants.Intake.wheelkD;
        rightWheelSlot0Config.kG = Constants.Intake.wheelkG;
        rightWheelSlot0Config.kV = Constants.Intake.wheelkV;
        rightWheelSlot0Config.kS = Constants.Intake.wheelkS;
        rightWheelSlot0Config.GravityType = GravityTypeValue.Elevator_Static;

        // Motion Magic setup
        motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = Constants.Intake.maxVelocity;
        motionMagicConfig.MotionMagicAcceleration = Constants.Intake.maxAcceleration;

        // Motion Magic setup
        wheelMotionMagicConfig = new MotionMagicConfigs();
        wheelMotionMagicConfig.MotionMagicCruiseVelocity = Constants.Intake.wheelMaxVelocity;
        wheelMotionMagicConfig.MotionMagicAcceleration = Constants.Intake.wheelMaxAcceleration;

        // Set the configurations
        leftWheelMotor.getConfigurator().apply(leftWheelConfig);
        leftWheelMotor.getConfigurator().apply(leftWheelSlot0Config);
        leftWheelMotor.getConfigurator().apply(wheelMotionMagicConfig);
        leftWheelMotor.setNeutralMode(NeutralModeValue.Brake);

        rightWheelMotor.getConfigurator().apply(rightWheelConfig);
        rightWheelMotor.getConfigurator().apply(rightWheelSlot0Config);
        rightWheelMotor.getConfigurator().apply(wheelMotionMagicConfig);
        rightWheelMotor.setNeutralMode(NeutralModeValue.Brake);
        rightWheelMotor.setControl(new Follower(Constants.Intake.leftWheelMotorID, true));

        intakeSensor = new CANrange(Constants.Intake.sensorID);
        intakeSensorConfig = new CANrangeConfiguration();
        intakeSensorConfig.ProximityParams.ProximityThreshold = Constants.Intake.detectionRange;
        intakeSensorConfig.ProximityParams.ProximityHysteresis = Constants.Intake.sensorDebounce;
        intakeSensorConfig.FovParams.FOVRangeX = Constants.Intake.fovXRange;
        intakeSensorConfig.FovParams.FOVRangeY = Constants.Intake.fovYRange;
        intakeSensorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        intakeSensor.getConfigurator().apply(intakeSensorConfig);

        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.getConfigurator().apply(PivotSlot0Config);
        pivotMotor.getConfigurator().apply(motionMagicConfig);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        intakePosition = pivotMotor.getPosition();
        intakeVelocity = pivotMotor.getVelocity();
        intakeAppliedVolts = pivotMotor.getMotorVoltage();
        intakeCurrent = pivotMotor.getStatorCurrent();

        wheelVelocityLeft = leftWheelMotor.getVelocity();
        wheelVelocityRight = rightWheelMotor.getVelocity();
        wheelAppliedVoltsLeft = leftWheelMotor.getMotorVoltage();
        wheelAppliedVoltsRight = rightWheelMotor.getMotorVoltage();
        wheelCurrentLeft = leftWheelMotor.getStatorCurrent();
        wheelCurrentRight = rightWheelMotor.getStatorCurrent();

        isGamePieceInIntake = intakeSensor.getIsDetected();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.Intake.ODOMETRY_FREQUENCY,
                intakeVelocity,
                intakePosition,
                intakeCurrent,
                intakeAppliedVolts,
                wheelVelocityLeft,
                wheelVelocityRight,
                wheelAppliedVoltsLeft,
                wheelVelocityRight,
                wheelCurrentLeft,
                wheelCurrentRight
        );

        BaseStatusSignal.setUpdateFrequencyForAll(50, isGamePieceInIntake);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.isGamePieceInIntake = isGamePieceInIntake.getValue();

        inputs.intakePosition = intakePosition.getValueAsDouble();
        inputs.intakeCurrent = intakeCurrent.getValueAsDouble();
        inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();

        inputs.intakeWheelLeftVelocityRotationsPerSec = wheelVelocityLeft.getValueAsDouble();
        inputs.intakeWheelLeftAppliedVolts = wheelAppliedVoltsLeft.getValueAsDouble();
        inputs.intakeWheelLeftCurrent = wheelCurrentLeft.getValueAsDouble();
        inputs.intakeWheelRightVelocityRotationsPerSec = wheelVelocityRight.getValueAsDouble();
        inputs.intakeWheelRightAppliedVolts = wheelAppliedVoltsRight.getValueAsDouble();
        inputs.intakeWheelRightCurrent = wheelCurrentRight.getValueAsDouble();

        inputs.intakeSetPosition = targetRotations;
        inputs.intakeWheelLeftWheelSetVelocity = targetLeftWheelVelocity;
        inputs.intakeWheelRightWheelSetVelocity = targetRightWheelVelocity;

        inputs.isIntakeAtSetpoint = MathUtil.isNear(targetRotations, inputs.intakePosition, Constants.Intake.pivotErrorTolerance);
        inputs.isIntakeWheelLeftAtSetpoint = MathUtil.isNear(targetLeftWheelVelocity, inputs.intakeWheelLeftWheelSetVelocity, Constants.Intake.wheelVelocityErrorTolerance);
        inputs.isIntakeWheelRightAtSetpoint = MathUtil.isNear(targetRightWheelVelocity, inputs.intakeWheelRightWheelSetVelocity, Constants.Intake.wheelVelocityErrorTolerance);

        inputs.isLeftWheelMotorTooHot = leftWheelMotor.getDeviceTemp().getValueAsDouble() >= 70;
        inputs.isRightWheelMotorTooHot = rightWheelMotor.getDeviceTemp().getValueAsDouble() >= 70;
    }

    @Override
    public void setIntakePosition(double position) {
        pivotMotor.setControl(new MotionMagicVoltage(position));
        targetRotations = position;
    }
    @Override
    public void setIntakeWheelRightVelocity(double velocity) {
        rightWheelMotor.setControl(new MotionMagicVelocityVoltage(velocity));
        targetRightWheelVelocity = velocity;
    }
    @Override
    public void setIntakeWheelLeftVelocity(double velocity) {
        leftWheelMotor.setControl(new MotionMagicVelocityVoltage(velocity));
        targetLeftWheelVelocity = velocity;
    }

    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    @Override
    public void setIntakeWheelSpeed(double speed) {
        leftWheelMotor.set(speed);
        rightWheelMotor.set(speed);
    }
}
