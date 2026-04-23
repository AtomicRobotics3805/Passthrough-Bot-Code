package org.firstinspires.ftc.teamcode

import dev.nextftc.core.units.m
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.GlobalVars.GOAL_POS
import org.firstinspires.ftc.teamcode.GlobalVars.SHOOTING
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

fun calculatePower(): Double {
    val currentPos = PedroComponent.follower.pose
    val distance = if (!LimeLight.ll.latestResult.fiducialResults.any { it.fiducialId == if (GlobalVars.RED_ALLIANCE) 24 else 20 } ) {
        if (GlobalVars.RED_ALLIANCE) {
            sqrt((GOAL_POS.mirror().x - currentPos.x).pow(2) + (GOAL_POS.mirror().y - currentPos.y).pow(2)) // Pythagoras
        } else {
            sqrt((GOAL_POS.x - currentPos.x).pow(2) + (GOAL_POS.y - currentPos.y).pow(2)) // Pythagoras
        }
    } else {
        abs(LimeLight.ll.latestResult.fiducialResults[0].robotPoseTargetSpace.position.z.m.inIn)
    }

    return if (SHOOTING) {
        (0.0937119 * distance.pow(2)) + (-2.1386 * distance) + 2042.09419
    } else {
        2800.0
    }
}