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
import frc.robot.subsystems.OuttakePivot.pivotMotor
import frc.robot.utils.PivotParameters.PIVOT_MAGIC_PINGU
import frc.robot.utils.PivotParameters.PIVOT_PINGU
import frc.robot.utils.PivotParameters.PIVOT_SOFT_LIMIT_DOWN
import frc.robot.utils.PivotParameters.PIVOT_SOFT_LIMIT_UP
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.emu.OuttakePivotState
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import xyz.malefic.frc.extension.configureWithDefaults
import xyz.malefic.frc.pingu.AlertPingu.add

object OuttakePivot : SubsystemBase() {
    private val pivotMotor: TalonFX = TalonFX(0)
    private val voltagePivotPos: PositionVoltage = PositionVoltage(0.0)
    private var posRequest: PositionDutyCycle
    private var velocityRequest: VelocityTorqueCurrentFOC
    private val voltageOut: VoltageOut
    private val motionMagicVoltage: MotionMagicVoltage
    private val cycleOut: DutyCycleOut

    init {
        pivotMotor.configureWithDefaults(
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
        pivotMotor.setPosition(0.0)

        add(pivotMotor, "pivot")
    }

    fun pivotDown() {
        pivotMotor.setControl(voltagePivotPos.withPosition(OuttakePivotState.DOWN.pos))
    }

    fun pivotUp() {
        pivotMotor.setControl(voltagePivotPos.withPosition(OuttakePivotState.UP.pos))
    }
}
