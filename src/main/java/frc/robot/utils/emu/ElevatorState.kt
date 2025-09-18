@file:Suppress("ktlint:standard:filename")

package frc.robot.utils.emu

// Enum class for the position of Elevator
enum class ElevatorState(
    @JvmField val pos: Double,
) {
    UP(65.0),
    UPPER_UP(48.75),
    MIDDLE(32.5),
    LOWER_DOWN(16.25),
    DOWN(0.0),
}
