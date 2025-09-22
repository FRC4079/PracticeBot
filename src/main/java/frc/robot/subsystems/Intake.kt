@file:Suppress("ktlint:standard:filename")

package frc.robot.subsystems

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Elevator.elevatorDown
import frc.robot.subsystems.Elevator.toBeSetState
import frc.robot.utils.IntakeParameters.INTAKE_MAGIC_PINGU
import frc.robot.utils.IntakeParameters.INTAKE_PINGU
import frc.robot.utils.IntakeParameters.INTAKE_SOFT_LIMIT_DOWN
import frc.robot.utils.IntakeParameters.INTAKE_SOFT_LIMIT_UP
import frc.robot.utils.emu.ElevatorState
import xyz.malefic.frc.pingu.AlertPingu.add

object Intake : SubsystemBase() {
    private val intakeStarMotor: TalonFX = TalonFX(0)
    private val intakeStarConfigs: TalonFXConfiguration = TalonFXConfiguration()
    private val intakeWheelMotor: TalonFX = TalonFX(0)
    private val intakeWheelConfigs: TalonFXConfiguration = TalonFXConfiguration()
    private val voltagePos: PositionVoltage = PositionVoltage(0.0)
    private var posStarRequest: PositionDutyCycle
    private var velocityStarRequest: VelocityTorqueCurrentFOC
    private var voltageStarOut: VoltageOut
    private var motionStarMagicVoltage: MotionMagicVoltage
    private var cycleStarOut: DutyCycleOut
    private var motionStarMagicConfigs: MotionMagicConfigs

    // separation
    private var posWheelRequest: PositionDutyCycle
    private var velocityWheelRequest: VelocityTorqueCurrentFOC
    private var voltageWheelOut: VoltageOut
    private var motionWheelMagicVoltage: MotionMagicVoltage
    private var cycleWheelOut: DutyCycleOut
    private var motionWheelMagicConfigs: MotionMagicConfigs

    init {
        intakeStarConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake
        intakeStarConfigs.Slot0.kP = INTAKE_PINGU.p
        intakeStarConfigs.Slot0.kI = INTAKE_PINGU.i
        intakeStarConfigs.Slot0.kD = INTAKE_PINGU.d
        intakeStarConfigs.Slot0.kV = INTAKE_PINGU.v!!
        intakeStarConfigs.Slot0.kS = INTAKE_PINGU.s!!
        intakeStarMotor.configurator.apply(intakeStarConfigs)
        val intakeMotorCurrentConfig = CurrentLimitsConfigs()
        val intakeMotorRampConfig = ClosedLoopRampsConfigs()
        val intakeSoftLimitConfig = SoftwareLimitSwitchConfigs()
        intakeMotorCurrentConfig.SupplyCurrentLimit = 40.79
        intakeMotorCurrentConfig.SupplyCurrentLimitEnable = true
        intakeStarMotor.configurator.apply(intakeMotorCurrentConfig)
        intakeMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.0
        intakeStarMotor.configurator.apply(intakeMotorRampConfig)
        intakeSoftLimitConfig.ForwardSoftLimitEnable = true
        intakeSoftLimitConfig.ReverseSoftLimitEnable = true
        intakeSoftLimitConfig.ForwardSoftLimitThreshold = INTAKE_SOFT_LIMIT_UP
        intakeSoftLimitConfig.ReverseSoftLimitThreshold = INTAKE_SOFT_LIMIT_DOWN
        intakeStarConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        intakeStarConfigs.SoftwareLimitSwitch = intakeSoftLimitConfig
        intakeStarConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.1
        intakeStarMotor.configurator.apply(intakeSoftLimitConfig)
        velocityStarRequest = VelocityTorqueCurrentFOC(0.0)
        posStarRequest = PositionDutyCycle(0.0)
        voltageStarOut = VoltageOut(0.0)
        motionStarMagicVoltage = MotionMagicVoltage(0.0)
        cycleStarOut = DutyCycleOut(0.0)
        motionStarMagicConfigs = intakeStarConfigs.MotionMagic
        motionStarMagicConfigs.MotionMagicCruiseVelocity = INTAKE_MAGIC_PINGU.velocity
        motionStarMagicConfigs.MotionMagicAcceleration = INTAKE_MAGIC_PINGU.acceleration
        motionStarMagicConfigs.MotionMagicJerk = INTAKE_MAGIC_PINGU.jerk
        motionStarMagicVoltage.Slot = 0
        velocityStarRequest.OverrideCoastDurNeutral = false

        voltageStarOut.OverrideBrakeDurNeutral = false
        voltageStarOut.EnableFOC = true

        cycleStarOut.EnableFOC = false
        intakeStarMotor.setPosition(0.0)

        intakeStarMotor.configurator.apply(intakeStarConfigs)

        intakeStarMotor.configurator.apply(motionStarMagicConfigs)

        add(intakeStarMotor, "intake")

        // separation between wheel and star

        intakeWheelConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake
        intakeWheelConfigs.Slot0.kP = INTAKE_PINGU.p
        intakeWheelConfigs.Slot0.kI = INTAKE_PINGU.i
        intakeWheelConfigs.Slot0.kD = INTAKE_PINGU.d
        intakeWheelConfigs.Slot0.kV = INTAKE_PINGU.v!!
        intakeWheelConfigs.Slot0.kS = INTAKE_PINGU.s!!
        intakeWheelMotor.configurator.apply(intakeWheelConfigs)
        val intakeWheelMotorCurrentConfig = CurrentLimitsConfigs()
        val intakeWheelMotorRampConfig = ClosedLoopRampsConfigs()
        val intakeWheelSoftLimitConfig = SoftwareLimitSwitchConfigs()
        intakeMotorCurrentConfig.SupplyCurrentLimit = 40.79
        intakeMotorCurrentConfig.SupplyCurrentLimitEnable = true
        intakeWheelMotor.configurator.apply(intakeMotorCurrentConfig)
        intakeMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.0
        intakeWheelMotor.configurator.apply(intakeMotorRampConfig)
        intakeSoftLimitConfig.ForwardSoftLimitEnable = true
        intakeSoftLimitConfig.ReverseSoftLimitEnable = true
        intakeSoftLimitConfig.ForwardSoftLimitThreshold = INTAKE_SOFT_LIMIT_UP
        intakeSoftLimitConfig.ReverseSoftLimitThreshold = INTAKE_SOFT_LIMIT_DOWN
        intakeWheelConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        intakeWheelConfigs.SoftwareLimitSwitch = intakeSoftLimitConfig
        intakeWheelConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.1
        intakeWheelMotor.configurator.apply(intakeSoftLimitConfig)
        velocityWheelRequest = VelocityTorqueCurrentFOC(0.0)
        posWheelRequest = PositionDutyCycle(0.0)
        voltageWheelOut = VoltageOut(0.0)
        motionWheelMagicVoltage = MotionMagicVoltage(0.0)
        cycleWheelOut = DutyCycleOut(0.0)
        motionWheelMagicConfigs = intakeWheelConfigs.MotionMagic
        motionWheelMagicConfigs.MotionMagicCruiseVelocity = INTAKE_MAGIC_PINGU.velocity
        motionWheelMagicConfigs.MotionMagicAcceleration = INTAKE_MAGIC_PINGU.acceleration
        motionWheelMagicConfigs.MotionMagicJerk = INTAKE_MAGIC_PINGU.jerk
        motionStarMagicVoltage.Slot = 0
        velocityStarRequest.OverrideCoastDurNeutral = false

        voltageStarOut.OverrideBrakeDurNeutral = false
        voltageStarOut.EnableFOC = true

        cycleStarOut.EnableFOC = false
        intakeWheelMotor.setPosition(0.0)

        intakeWheelMotor.configurator.apply(intakeWheelConfigs)

        intakeWheelMotor.configurator.apply(motionStarMagicConfigs)

        add(intakeWheelMotor, "intake")
    }

    override fun periodic() {
        intakeStarMotor.set(1.0)
        intakeWheelMotor.set(1.0)
    }

    fun shootCoral() {
        Elevator.elevatorMove(toBeSetState)
    }
}
