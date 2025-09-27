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
import frc.robot.subsystems.Intake.motorShoot
import frc.robot.subsystems.Intake.stopMotor
import frc.robot.utils.PivotParameters.PIVOT_MAGIC_PINGU
import frc.robot.utils.PivotParameters.PIVOT_PINGU
import frc.robot.utils.PivotParameters.PIVOT_SOFT_LIMIT_DOWN
import frc.robot.utils.PivotParameters.PIVOT_SOFT_LIMIT_UP
import frc.robot.utils.emu.AlgaeState
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.emu.OuttakeShooterState
import xyz.malefic.frc.extension.configureWithDefaults
import xyz.malefic.frc.pingu.AlertPingu.add
import java.lang.Thread.sleep

object OuttakePivot : SubsystemBase() {
    val pivotMotor: TalonFX = TalonFX(0)
    val voltagePivotPos: PositionVoltage = PositionVoltage(0.0)
    private var posRequest: PositionDutyCycle
    private var velocityRequest: VelocityTorqueCurrentFOC
    private val voltageOut: VoltageOut
    private val motionMagicVoltage: MotionMagicVoltage
    private val cycleOut: DutyCycleOut
    private val algaeSensor = DigitalInput(1)
    private var intakingAlgae = false

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

    /*
    fun pivotDown() {
        pivotMotor.setControl(voltagePivotPos.withPosition(OuttakePivotState.DOWN.pos))
    }

    fun pivotUp() {
        pivotMotor.setControl(voltagePivotPos.withPosition(OuttakePivotState.UP.pos))
    }
     */
    fun intakeAlgae() {
        intakingAlgae = true
    }

    fun shootMotor() {
        Elevator.elevatorMove(ElevatorState.L4)
        pivotMotor.setControl(voltagePivotPos.withPosition(AlgaeState.UP.pos))
        sleep(2000)
        pivotMotor.setControl(voltagePivotPos.withPosition(AlgaeState.DOWN.pos))
        Elevator.elevatorMove(ElevatorState.L0)
    }

    fun shootCoral() {
        val shoot = OuttakeShooterState.FORWARD
        Elevator.elevatorMove(Elevator.toBeSetState)
        pivotMotor.setControl(voltagePivotPos.withPosition(AlgaeState.DOWN.pos))
        motorShoot(state = shoot)
        stopMotor()
        pivotMotor.setControl(voltagePivotPos.withPosition(AlgaeState.UP.pos))
        Elevator.elevatorMove(ElevatorState.L0)
    }

    override fun periodic() {
        if (intakingAlgae) {
            pivotMotor.setControl(voltagePivotPos.withPosition(AlgaeState.DOWN.pos))
            val ifAlgae = algaeSensor.get()
            if (ifAlgae) {
                pivotMotor.stopMotor()
            } else {
                pivotMotor.setControl(voltagePivotPos.withPosition(AlgaeState.MIDDLE.pos))
                sleep(2000)
                pivotMotor.setControl(voltagePivotPos.withPosition(AlgaeState.DOWN.pos))
            }
        }
    }
}
