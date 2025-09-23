package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.OuttakePivot.pivotMotor
import frc.robot.subsystems.OuttakePivot.voltagePivotPos
import frc.robot.utils.emu.AlgaeState
import frc.robot.utils.emu.ElevatorState

object ShootCoral : SubsystemBase() {
    fun shootCoral() {
        Elevator.elevatorMove(state = Elevator.toBeSetState)
        pivotMotor.setControl(voltagePivotPos.withPosition(AlgaeState.DOWN.pos))
    }
}
