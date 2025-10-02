package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ArmIOHardware implements ArmIO{

    protected final TalonFX armMotor;
    private final CANcoder armEncoder;
    private final TalonFXConfiguration armMotorConfig;
    private final Slot0Configs slot0Config;
    private final MotionMagicConfigs motionMagicConfig;
    private final CANcoderConfiguration armEncoderConfig;

    private final MotionMagicVoltage armMotorRequest = new MotionMagicVoltage(0.0);

    private double targetRotations;
    private final StatusSignal<Angle> armPosition;
    private final StatusSignal<AngularVelocity> armVelocity;
    private final StatusSignal<Voltage> armAppliedVolts;
    private final StatusSignal<Current> armCurrent;

    public ArmIOHardware() {
        // Motor basic setup
        armMotor = new TalonFX(Constants.Arm.ArmMotorID);
        armEncoder = new CANcoder(Constants.Arm.ArmEncoderID);
        armMotorConfig = new TalonFXConfiguration();
        armMotorConfig.Feedback.SensorToMechanismRatio = Constants.Arm.conversionFactor;
        armMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        armMotorConfig.Feedback.FeedbackRemoteSensorID = Constants.Arm.ArmEncoderID;
        armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Arm.rotationsMax;
        armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Arm.rotationsMin;
        armMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Slot 0 PID setup
        slot0Config = new Slot0Configs();
        slot0Config.kP = Constants.Arm.kP;
        slot0Config.kI = Constants.Arm.kI;
        slot0Config.kD = Constants.Arm.kD;
        slot0Config.kG = Constants.Arm.kG;
        slot0Config.kV = Constants.Arm.kV;
        slot0Config.kS = Constants.Arm.kS;
        slot0Config.GravityType = GravityTypeValue.Arm_Cosine;

        // Motion Magic setup
        motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = Constants.Arm.maxVelocity;
        motionMagicConfig.MotionMagicAcceleration = Constants.Arm.maxAcceleration;

        armEncoderConfig = new CANcoderConfiguration();
        armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        armEncoder.getConfigurator().apply(armEncoderConfig);

        // Set the configurations
        armMotor.getConfigurator().apply(armMotorConfig);
        armMotor.getConfigurator().apply(slot0Config);
        armMotor.getConfigurator().apply(motionMagicConfig);
        armMotor.setNeutralMode(NeutralModeValue.Brake);

        armPosition = armMotor.getPosition();
        armVelocity = armMotor.getVelocity();
        armAppliedVolts = armMotor.getMotorVoltage();
        armCurrent = armMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(Constants.Arm.ODOMETRY_FREQUENCY, armPosition, armVelocity, armCurrent, armAppliedVolts);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(armPosition, armVelocity, armCurrent, armAppliedVolts);
        inputs.armPositionRotations = armPosition.getValueAsDouble();
        inputs.armVelocityRotationsPerSec = armVelocity.getValueAsDouble();
        inputs.armCurrent = armCurrent.getValueAsDouble();
        inputs.armAppliedVolts = armAppliedVolts.getValueAsDouble();
        inputs.armSetPoint = armMotorRequest.Position;
        inputs.isArmAtSetpoint = armMotor.getClosedLoopError().getValueAsDouble() < Constants.Arm.errorTolerance;
        inputs.motionMagicError = armMotor.getClosedLoopError().getValueAsDouble();
    }

    @Override
    public void setArmSpeed(double speed) {
        armMotor.set(speed);
    }

    @Override
    public void setArmPosition(double position) {
        armMotor.setControl(armMotorRequest.withPosition(position));
    }
}
