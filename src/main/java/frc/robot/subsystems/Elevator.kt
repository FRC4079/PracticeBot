package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.PivotParameters.PIVOT_MAGIC_PINGU
import frc.robot.utils.PivotParameters.PIVOT_PINGU
import frc.robot.utils.PivotParameters.PIVOT_SOFT_LIMIT_DOWN
import frc.robot.utils.PivotParameters.PIVOT_SOFT_LIMIT_UP
import frc.robot.utils.emu.ElevatorState
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import xyz.malefic.frc.extension.configureWithDefaults
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
    private var elevatorP: LoggedNetworkNumber? = null
    private var elevatorI: LoggedNetworkNumber? = null
    private var elevatorD: LoggedNetworkNumber? = null
    private var elevatorV: LoggedNetworkNumber? = null
    private var elevatorS: LoggedNetworkNumber? = null
    private var elevatorG: LoggedNetworkNumber? = null
    private var cruiseV: LoggedNetworkNumber? = null
    private var acc: LoggedNetworkNumber? = null
    private var jerk: LoggedNetworkNumber? = null
    lateinit var toBeSetState: ElevatorState

    init {
        elevatorMotor.configureWithDefaults(
            PIVOT_PINGU,
            InvertedValue.CounterClockwise_Positive,
            limitThresholds = PIVOT_SOFT_LIMIT_UP to PIVOT_SOFT_LIMIT_DOWN,
            dutyCycleNeutralDeadband = 0.1,
            motionMagicPingu = PIVOT_MAGIC_PINGU,
        )
        velocityRequest = VelocityTorqueCurrentFOC(0.0)
        posRequest = PositionDutyCycle(0.0)
        voltageOut = VoltageOut(0.0)
        motionMagicVoltage = MotionMagicVoltage(0.0)
        cycleOut = DutyCycleOut(0.0)
        motionMagicVoltage.Slot = 0
        velocityRequest.OverrideCoastDurNeutral = false

        voltageOut.OverrideBrakeDurNeutral = false
        voltageOut.EnableFOC = true

        cycleOut.EnableFOC = false
        elevatorMotor.setPosition(0.0)

        elevatorMotor.configurator.apply(elevatorConfigs)

        add(elevatorMotor, "elevator")
    }

    fun setToBeSetState(level: ElevatorState) {
        toBeSetState = level
    }

    fun elevatorMove(state: ElevatorState) {
        elevatorMotor.setControl(voltagePos.withPosition(state.pos))
    }
}
