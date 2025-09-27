package frc.robot

import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Elevator.toBeSetState
import frc.robot.subsystems.Intake
import frc.robot.subsystems.OuttakePivot
import frc.robot.utils.emu.ElevatorState
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import xyz.malefic.frc.emu.Button
import xyz.malefic.frc.emu.Button.DPAD_UP
import xyz.malefic.frc.emu.Button.LEFT_BUMPER
import xyz.malefic.frc.emu.Button.RIGHT_BUMPER
import xyz.malefic.frc.emu.Button.RIGHT_STICK
import xyz.malefic.frc.pingu.Bingu.bindings

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {
    val pad: XboxController = XboxController(1)

    var networkChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser<Command?>("AutoChooser")

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {

        networkChooser.addDefaultOption("Do Nothing", PathPlannerAuto("Straight Auto"))

        configureBindings()
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [frc.robot.utils.controller.Trigger] or our [JoystickButton] constructor with an arbitrary predicate, or via
     * the named factories in [edu.wpi.first.wpilibj2.command.button.CommandGenericHID]'s subclasses for [edu.wpi.first.wpilibj2.command.button.CommandXboxController]/[edu.wpi.first.wpilibj2.command.button.CommandPS4Controller] controllers or [edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        pad.bindings {
            // tasks separated by difficulty
            press(Button.LEFT_STICK) {
                // first task
                Intake.intakeCoral()
            }

            //

            press(DPAD_UP) {
                Elevator.setToBeSetState(level = ElevatorState.L4)
                toBeSetState = ElevatorState.L4
            }
            press(Button.DPAD_DOWN) {
                Elevator.setToBeSetState(level = ElevatorState.L0)
                toBeSetState = ElevatorState.L0
            }
            press(Button.DPAD_RIGHT) {
                Elevator.setToBeSetState(level = ElevatorState.L3)
                toBeSetState = ElevatorState.L3
            }
            press(Button.DPAD_LEFT) {
                Elevator.setToBeSetState(level = ElevatorState.L2)
                toBeSetState = ElevatorState.L2
            }
            press(Button.Y) {
                Elevator.setToBeSetState(level = ElevatorState.L1)
                toBeSetState = ElevatorState.L1
            }
            //

            press(RIGHT_STICK) {
                // second task
                OuttakePivot.shootMotor()
            }

            //

            press(RIGHT_BUMPER) {
                // fourth task
                OuttakePivot.shootCoral()
            }

            //

            press(LEFT_BUMPER) {
                // third task
                OuttakePivot.intakeAlgae()
            }
        }
    }

    val autonomousCommand: Command?
        get() = networkChooser.get()
}
