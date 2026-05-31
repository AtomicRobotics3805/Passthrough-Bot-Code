package org.firstinspires.ftc.teamcode

import com.pedropathing.geometry.Pose
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.m
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.GlobalVars.SHOOTING
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

object AutoAimCalcs {

    val goalPos = Pose(16.3, 131.8)

    var pose = Pose()

    fun calculatePower(shooting: Boolean): Double {
        val currentPos = PedroComponent.follower.pose
        var distance = 0.0

        distance = if (!LimeLight.ll.latestResult.fiducialResults.any { it.fiducialId == if (GlobalVars.RED_ALLIANCE) 24 else 20 } ) {
            if (GlobalVars.RED_ALLIANCE) {
                sqrt(((goalPos.mirror().x - currentPos.x) * (goalPos.mirror().x - currentPos.x)) + ((goalPos.mirror().y - currentPos.y) * (goalPos.mirror().y - currentPos.y)))
            } else {
                sqrt(((goalPos.x - currentPos.x) * (goalPos.x - currentPos.x)) + ((goalPos.y - currentPos.y) * (goalPos.y - currentPos.y)))
            }
        } else {
            abs(LimeLight.ll.latestResult.fiducialResults[0].robotPoseTargetSpace.position.z.m.inIn)
        }


        return if (shooting) {
            3805.0 //(0.0937119*(distance*distance)) + (-2.1386 * distance) + 2042.09419
        } else {
            1000.0
        }
    }

    fun calculateAimAngle(): Double {
        val currentPos = PedroComponent.follower.pose



        if (LimeLight.ll.latestResult.fiducialResults.any { it.fiducialId == if (GlobalVars.RED_ALLIANCE) 24 else 20 } ) {
            if (PedroComponent.follower.pose.y <= 48.0) {
                val heading = PedroComponent.follower.heading - LimeLight.getTX() + if (GlobalVars.RED_ALLIANCE) { 1.deg.inRad } else { -1.deg.inRad }
                pose = PedroComponent.follower.pose.withHeading(heading)
            } else if (PedroComponent.follower.pose.y >= 130.0) {
                val heading = PedroComponent.follower.heading - LimeLight.getTX() + if (GlobalVars.RED_ALLIANCE) { -1.deg.inRad } else { 1.deg.inRad }
                pose = PedroComponent.follower.pose.withHeading(heading)
            } else {
                val heading = PedroComponent.follower.heading - LimeLight.getTX()
                pose = PedroComponent.follower.pose.withHeading(heading)
            }
        }

        if (PedroComponent.follower.pose.distanceFrom(pose) < 10.0) {
            return pose.heading
        } else {
            if (GlobalVars.RED_ALLIANCE) {
                val dx = 144 - currentPos.x
                val dy = 144 - currentPos.y

                return atan2(dy, dx)
            } else {
                val dx = 0 - currentPos.x
                val dy = 144 - currentPos.y

                return atan2(dy, dx)
            }
        }
    }

}

class PIDJoystickBlend(val joystick: Supplier<Double>, val pid: Supplier<Double>): Supplier<Double> {
    var usePID = false

    override fun get(): Double {
        return if (usePID) pid.get() else joystick.get()
    }
}