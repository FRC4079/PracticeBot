package frc.robot.subsystems

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.OuttakeShooter.shooterMotor
import frc.robot.utils.PivotParameters.PIVOT_MAGIC_PINGU
import frc.robot.utils.PivotParameters.PIVOT_PINGU
import frc.robot.utils.PivotParameters.PIVOT_SOFT_LIMIT_DOWN
import frc.robot.utils.PivotParameters.PIVOT_SOFT_LIMIT_UP
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.emu.OuttakeShooterState
import xyz.malefic.frc.extension.configureWithDefaults
import xyz.malefic.frc.pingu.AlertPingu.add

object OuttakeShooter : SubsystemBase() {
    private val shooterMotor: TalonFX = TalonFX(0)
    private val voltageShooterPos: PositionVoltage = PositionVoltage(0.0)
    private val posRequest: PositionDutyCycle
    private val velocityRequest: VelocityTorqueCurrentFOC
    private val voltageOut: VoltageOut
    private val motionMagicVoltage: MotionMagicVoltage
    private val cycleOut: DutyCycleOut
    private val coralSensor = DigitalInput(1)
    private var intakingCoral = false

    init {
        shooterMotor.configureWithDefaults(
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
        shooterMotor.setPosition(0.0)

        add(shooterMotor, "shooter")
    }

    override fun periodic() {
        if (intakingCoral) {
            val ifCoral = coralSensor.get()
            if (!ifCoral) {
                shooterMotor.stopMotor()
                intakingCoral = false
            } else {
                shooterMotor.setControl(voltageShooterPos.withPosition(OuttakeShooterState.FORWARD.pos))
            }
        }
    }

    fun intakeCoral() {
        intakingCoral = true
    }

    fun shootMotor() {
        Elevator.elevatorMove(ElevatorState.L4)
        shooterMotor.setControl(voltageShooterPos.withPosition(65.0))
    }
}
