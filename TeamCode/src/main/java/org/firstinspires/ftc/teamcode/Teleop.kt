package org.firstinspires.ftc.teamcode

import android.graphics.Path
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.ftc.localization.constants.PinpointConstants
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.AngleType
import dev.nextftc.control.feedback.AngularFeedback
import dev.nextftc.control.feedback.FeedbackType
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedback.PIDElement
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.components.LoopTimeComponent
import dev.nextftc.hardware.driving.FieldCentric
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower
import kotlin.compareTo
import kotlin.math.abs
@Configurable
@TeleOp(name = "Teleop")
class Teleop : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(Intake, Shooter, Gate, LimeLight),
            BulkReadComponent,
            PedroComponent(Constants::createFollower),
            BindingsComponent,
            LoopTimeComponent()
        )
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
    }

    companion object {
        @JvmField
        var mainCoefficients = PIDCoefficients(-0.8, 0.0, 0.08)

        @JvmField
        var secondaryCoefficients = PIDCoefficients(-1.0, 0.0, 0.2)

        @JvmField
        var pidSwitch = 30.0


        @JvmField
        val mainController = controlSystem {
            feedback(
                AngularFeedback(
                    AngleType.RADIANS,
                    PIDElement(
                        FeedbackType.POSITION,
                        mainCoefficients
                    )
                )
            )
        }

        @JvmField
        val secondaryController = controlSystem {
            feedback(
                AngularFeedback(
                    AngleType.RADIANS,
                    PIDElement(
                        FeedbackType.POSITION,
                        secondaryCoefficients
                    )
                )
            )
        }

        fun calculateHeadingPID(): Double {
            if (abs(AutoAimCalcs.calculateAimAngle() - PedroComponent.follower.heading) < pidSwitch.deg.inRad) {
                secondaryController.goal = KineticState(AutoAimCalcs.calculateAimAngle())

                return secondaryController.calculate(KineticState(PedroComponent.follower.heading))
            } else {
                mainController.goal = KineticState(AutoAimCalcs.calculateAimAngle())

                return mainController.calculate(KineticState(PedroComponent.follower.heading))
            }
        }

        val autoAimPID = PIDJoystickBlend(Gamepads.gamepad1.rightStickX::get, ::calculateHeadingPID)

        val pinpoint: GoBildaPinpointDriver by lazy { ActiveOpMode.hardwareMap.get(GoBildaPinpointDriver::class.java, "pinpoint") }
    }

    override fun onInit() {
        LimeLight.initialize()
    }

    private val frontLeftMotor = MotorEx("LF").brakeMode()
    private val frontRightMotor = MotorEx("RF").brakeMode()
    private val backLeftMotor = MotorEx("LB").brakeMode()
    private val backRightMotor = MotorEx("RB").brakeMode()


    override fun onStartButtonPressed() {
        LimeLight.autoRelocalize = true
        PedroComponent.follower.pose = GlobalVars.FINAL_AUTO_POS
        Gate.close()

        val driverControlled = MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
            -Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            autoAimPID,
            FieldCentric { pinpoint.getHeading(AngleUnit.RADIANS).rad }
        )

        driverControlled()



        Gamepads.gamepad1.rightTrigger.asButton { it >= 0.5 }.whenBecomesTrue { Intake.start() }.whenBecomesFalse { Intake.slowIn() }

        Gamepads.gamepad1.leftTrigger.asButton { it >= 0.5 }.whenBecomesTrue { Intake.reverse() }.whenBecomesFalse { Intake.slowIn() }

        Gamepads.gamepad1.leftBumper.whenBecomesTrue { MechanismRoutines.BeginShoot() }.whenBecomesFalse { MechanismRoutines.EndShoot() }



    }






}