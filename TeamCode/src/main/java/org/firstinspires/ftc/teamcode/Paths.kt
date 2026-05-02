package org.firstinspires.ftc.teamcode

import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.units.deg

object Paths {

    //region Poses

    val goalStartPos = Pose(16.9, 119.0, 144.1.deg.inRad)
    val farStartPos = Pose(53.0, 8.9, 90.deg.inRad)

    val nearShootPos = Pose(55.5, 87.25, 144.1.deg.inRad)
    val farShootPos = Pose(58.5, 18.5, 155.deg.inRad)

    val spikeMark1PosPre = Pose(41.0, 84.0, 180.deg.inRad)
    val spikeMark1PosPost = Pose(24.5, 84.0, 180.deg.inRad)

    val spikeMark2PosPre = Pose(41.0, 60.0, 180.deg.inRad)
    val spikeMark2PosPost = Pose(24.5, 60.0, 180.deg.inRad)

    val spikeMark3PosPre = Pose(41.0, 35.0, 180.deg.inRad)
    val spikeMark3PosPost = Pose(24.5, 35.0, 180.deg.inRad)

    val humanPlayerPosPre = Pose(16.0, 7.7, 180.deg.inRad)
    val humanPlayerPosPost = Pose(10.7, 7.7, 180.deg.inRad)

    val dumpPose = Pose(15.8, 70.0, 180.deg.inRad)
    val gateBezierControlPoint = Pose(27.2, 69.1)

    val nearParkPos = Pose(64.5, 96.0, 144.1.deg.inRad)
    val farParkPos = Pose (54.0, 26.5, 115.deg.inRad)

    //endregion

    //region Path Initialization

    lateinit var goalStartToCloseShoot: PathChain
    lateinit var farStartToFarShoot: PathChain

    lateinit var intakeSpike1: PathChain
    lateinit var intakeSpike2: PathChain
    lateinit var intakeSpike3: PathChain
    lateinit var intakeHumanPlayer: PathChain

    lateinit var spike1ToDump: PathChain
    lateinit var spike2ToDump: PathChain

    lateinit var closeShootToSpike1: PathChain
    lateinit var closeShootToSpike2: PathChain
    lateinit var farShootToSpike3: PathChain
    lateinit var farShootToHumanPlayer: PathChain

    lateinit var dumpToCloseShoot: PathChain
    lateinit var spike3ToFarShoot: PathChain
    lateinit var humanPlayerToFarShoot: PathChain

    lateinit var closeShootToClosePark: PathChain
    lateinit var farShootToFarPark: PathChain

    //endregion

    //region Path Definition


    //endregion

}