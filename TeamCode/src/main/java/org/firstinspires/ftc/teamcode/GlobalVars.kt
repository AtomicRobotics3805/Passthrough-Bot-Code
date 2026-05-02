package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose

@Configurable
object GlobalVars {


    val GOAL_POS = Pose(16.3, 131.8)

    @JvmField
    var RED_ALLIANCE = false

    @JvmField
    var SHOOTING = false

    @JvmField
    var SHOOT_1_BALL_DELAY = 1.0

    @JvmField
    var SHOOT_3_BALL_DELAY = 1.0
}