@file:Suppress("ktlint:standard:filename")

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
import frc.robot.utils.IntakeParameters.INTAKE_MAGIC_PINGU
import frc.robot.utils.IntakeParameters.INTAKE_PINGU
import frc.robot.utils.IntakeParameters.INTAKE_SOFT_LIMIT_DOWN
import frc.robot.utils.IntakeParameters.INTAKE_SOFT_LIMIT_UP
import frc.robot.utils.PivotParameters.PIVOT_SOFT_LIMIT_DOWN
import frc.robot.utils.PivotParameters.PIVOT_SOFT_LIMIT_UP
import frc.robot.utils.ShootParameters.SHOOT_MAGIC_PINGU
import frc.robot.utils.ShootParameters.SHOOT_PINGU
import frc.robot.utils.emu.OuttakeShooterState
import xyz.malefic.frc.extension.configureWithDefaults
import xyz.malefic.frc.pingu.AlertPingu.add

object Intake : SubsystemBase() {
    private val shooterMotor: TalonFX = TalonFX(0)
    private val intakeStarMotor: TalonFX = TalonFX(0)
    private val intakeWheelMotor: TalonFX = TalonFX(0)
    private var posStarRequest: PositionDutyCycle
    private var velocityStarRequest: VelocityTorqueCurrentFOC
    private var voltageStarOut: VoltageOut
    private var motionStarMagicVoltage: MotionMagicVoltage
    private var cycleStarOut: DutyCycleOut
    private var intakingCoral = true
    private val voltageShooterPos: PositionVoltage = PositionVoltage(0.0)
    private val posRequest: PositionDutyCycle
    private val velocityRequest: VelocityTorqueCurrentFOC
    private val voltageOut: VoltageOut
    private val motionMagicVoltage: MotionMagicVoltage
    private val cycleOut: DutyCycleOut

    // separation
    private var posWheelRequest: PositionDutyCycle
    private var velocityWheelRequest: VelocityTorqueCurrentFOC
    private var voltageWheelOut: VoltageOut
    private var motionWheelMagicVoltage: MotionMagicVoltage
    private var cycleWheelOut: DutyCycleOut

    //
    private val coralSensor = DigitalInput(1)

    init {
        shooterMotor.configureWithDefaults(
            SHOOT_PINGU,
            InvertedValue.CounterClockwise_Positive,
            dutyCycleNeutralDeadband = 0.1,
            motionMagicPingu = SHOOT_MAGIC_PINGU,
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
        intakeStarMotor.configureWithDefaults(
            INTAKE_PINGU,
            InvertedValue.CounterClockwise_Positive,
            limitThresholds = INTAKE_SOFT_LIMIT_UP to INTAKE_SOFT_LIMIT_DOWN,
            dutyCycleNeutralDeadband = 0.1,
            motionMagicPingu = INTAKE_MAGIC_PINGU,
        )
        velocityStarRequest = VelocityTorqueCurrentFOC(0.0)
        posStarRequest = PositionDutyCycle(0.0)
        voltageStarOut = VoltageOut(0.0)
        motionStarMagicVoltage = MotionMagicVoltage(0.0)
        cycleStarOut = DutyCycleOut(0.0)
        motionStarMagicVoltage.Slot = 0
        velocityStarRequest.OverrideCoastDurNeutral = false

        voltageStarOut.OverrideBrakeDurNeutral = false
        voltageStarOut.EnableFOC = true

        cycleStarOut.EnableFOC = false
        intakeStarMotor.setPosition(0.0)

        add(intakeStarMotor, "intake")

        // separation between wheel and star

        intakeWheelMotor.configureWithDefaults(
            INTAKE_PINGU,
            InvertedValue.CounterClockwise_Positive,
            limitThresholds = PIVOT_SOFT_LIMIT_UP to PIVOT_SOFT_LIMIT_DOWN,
            dutyCycleNeutralDeadband = 0.1,
            motionMagicPingu = INTAKE_MAGIC_PINGU,
        )
        velocityWheelRequest = VelocityTorqueCurrentFOC(0.0)
        posWheelRequest = PositionDutyCycle(0.0)
        voltageWheelOut = VoltageOut(0.0)
        motionWheelMagicVoltage = MotionMagicVoltage(0.0)
        cycleWheelOut = DutyCycleOut(0.0)
        motionStarMagicVoltage.Slot = 0
        velocityStarRequest.OverrideCoastDurNeutral = false

        voltageStarOut.OverrideBrakeDurNeutral = false
        voltageStarOut.EnableFOC = true

        cycleStarOut.EnableFOC = false
        intakeWheelMotor.setPosition(0.0)

        add(intakeWheelMotor, "intake")
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

    fun motorShoot(state: OuttakeShooterState) {
        shooterMotor.setControl(voltageShooterPos.withPosition(state.pos))
    }

    fun stopMotor() {
        shooterMotor.stopMotor()
    }
}
