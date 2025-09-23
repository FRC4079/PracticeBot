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
import frc.robot.subsystems.Elevator.elevatorMotor
import frc.robot.utils.ElevatorParameters.ELEVATOR_MAGIC_PINGU
import frc.robot.utils.ElevatorParameters.ELEVATOR_PINGU
import frc.robot.utils.ElevatorParameters.ELEVATOR_SOFT_LIMIT_DOWN
import frc.robot.utils.ElevatorParameters.ELEVATOR_SOFT_LIMIT_UP
import frc.robot.utils.emu.ElevatorState
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import xyz.malefic.frc.pingu.AlertPingu.add

object Elevator : SubsystemBase() {
    val elevatorMotor: TalonFX = TalonFX(0)
    private val elevatorConfigs: TalonFXConfiguration = TalonFXConfiguration()
    private val voltagePos: PositionVoltage = PositionVoltage(0.0)
    private val posRequest: PositionDutyCycle
    private val velocityRequest: VelocityTorqueCurrentFOC
    private val voltageOut: VoltageOut
    private val motionMagicVoltage: MotionMagicVoltage
    private val cycleOut: DutyCycleOut
    private var motionMagicConfigs: MotionMagicConfigs
    private var elevatorP: LoggedNetworkNumber? = null
    private var elevatorI: LoggedNetworkNumber? = null
    private var elevatorD: LoggedNetworkNumber? = null
    private var elevatorV: LoggedNetworkNumber? = null
    private var elevatorS: LoggedNetworkNumber? = null
    private var elevatorG: LoggedNetworkNumber? = null
    private var cruiseV: LoggedNetworkNumber? = null
    private var acc: LoggedNetworkNumber? = null
    private var jerk: LoggedNetworkNumber? = null
    public lateinit var toBeSetState: ElevatorState

    init {
        elevatorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake
        elevatorConfigs.Slot0.kP = ELEVATOR_PINGU.p
        elevatorConfigs.Slot0.kI = ELEVATOR_PINGU.i
        elevatorConfigs.Slot0.kD = ELEVATOR_PINGU.d
        elevatorConfigs.Slot0.kV = ELEVATOR_PINGU.v!!
        elevatorConfigs.Slot0.kS = ELEVATOR_PINGU.s!!
        elevatorMotor.configurator.apply(elevatorConfigs)
        val elevatorMotorCurrentConfig = CurrentLimitsConfigs()
        val elevatorMotorRampConfig = ClosedLoopRampsConfigs()
        val elevatorSoftLimitConfig = SoftwareLimitSwitchConfigs()
        elevatorMotorCurrentConfig.SupplyCurrentLimit = 40.79
        elevatorMotorCurrentConfig.SupplyCurrentLimitEnable = true
        elevatorMotor.configurator.apply(elevatorMotorCurrentConfig)
        elevatorMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.0
        elevatorMotor.configurator.apply(elevatorMotorRampConfig)
        elevatorSoftLimitConfig.ForwardSoftLimitEnable = true
        elevatorSoftLimitConfig.ReverseSoftLimitEnable = true
        elevatorSoftLimitConfig.ForwardSoftLimitThreshold = ELEVATOR_SOFT_LIMIT_UP
        elevatorSoftLimitConfig.ReverseSoftLimitThreshold = ELEVATOR_SOFT_LIMIT_DOWN
        elevatorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        elevatorConfigs.SoftwareLimitSwitch = elevatorSoftLimitConfig
        elevatorConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.1
        elevatorMotor.configurator.apply(elevatorSoftLimitConfig)
        velocityRequest = VelocityTorqueCurrentFOC(0.0)
        posRequest = PositionDutyCycle(0.0)
        voltageOut = VoltageOut(0.0)
        motionMagicVoltage = MotionMagicVoltage(0.0)
        cycleOut = DutyCycleOut(0.0)
        motionMagicConfigs = elevatorConfigs.MotionMagic
        motionMagicConfigs.MotionMagicCruiseVelocity = ELEVATOR_MAGIC_PINGU.velocity
        motionMagicConfigs.MotionMagicAcceleration = ELEVATOR_MAGIC_PINGU.acceleration
        motionMagicConfigs.MotionMagicJerk = ELEVATOR_MAGIC_PINGU.jerk
        motionMagicVoltage.Slot = 0
        velocityRequest.OverrideCoastDurNeutral = false

        voltageOut.OverrideBrakeDurNeutral = false
        voltageOut.EnableFOC = true

        cycleOut.EnableFOC = false
        elevatorMotor.setPosition(0.0)

        elevatorMotor.configurator.apply(elevatorConfigs)

        elevatorMotor.configurator.apply(motionMagicConfigs)

        add(elevatorMotor, "elevator")
        initializeLoggedNetworkPID()
    }

    fun setToBeSetState(level: ElevatorState) {
        toBeSetState = level
    }

    fun initializeLoggedNetworkPID() {
        elevatorP =
            LoggedNetworkNumber("Tuning/elevator/elevator P", elevatorConfigs.Slot0.kP)
        elevatorI =
            LoggedNetworkNumber("Tuning/elevator/elevator I", elevatorConfigs.Slot0.kI)
        elevatorD =
            LoggedNetworkNumber("Tuning/elevator/elevator D", elevatorConfigs.Slot0.kD)
        elevatorV =
            LoggedNetworkNumber("Tuning/elevator/elevator V", elevatorConfigs.Slot0.kV)
        elevatorS =
            LoggedNetworkNumber("Tuning/elevator/elevator S", elevatorConfigs.Slot0.kS)
        elevatorG =
            LoggedNetworkNumber("Tuning/elevator/elevator G", elevatorConfigs.Slot0.kG)

        cruiseV =
            LoggedNetworkNumber(
                "Tuning/elevator/MM Cruise Velocity",
                motionMagicConfigs.MotionMagicCruiseVelocity,
            )
        acc =
            LoggedNetworkNumber(
                "Tuning/elevator/MM Acceleration",
                motionMagicConfigs.MotionMagicAcceleration,
            )
        jerk =
            LoggedNetworkNumber(
                "Tuning/elevator/MM Jerk",
                @Suppress("ktlint:standard:max-line-length")
                motionMagicConfigs.MotionMagicJerk,
            )
    }

    fun elevatorMove(state: ElevatorState) {
        elevatorMotor.setControl(voltagePos.withPosition(state.pos))
    }
}
