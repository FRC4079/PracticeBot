package frc.robot.utils.emu

enum class OuttakeShooterState(
    @JvmField val pos: Double,
) {
    FORWARD(1.0),
    BACKWARD(-1.0),
}
