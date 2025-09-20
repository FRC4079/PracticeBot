@file:Suppress("ktlint:standard:filename")

package frc.robot.utils.emu

// Enum class for the position of Elevator
enum class ElevatorState(
    @JvmField val pos: Double,
) {
    L4(65.0),
    L3(48.75),
    L2(32.5),
    L1(16.25),
    L0(0.0),
}
