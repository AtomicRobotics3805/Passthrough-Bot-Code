package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.units.deg
import org.firstinspires.ftc.teamcode.GlobalVars.RED_ALLIANCE

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
    val dumpBezierControlPoint = Pose(27.2, 69.1)

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

    fun buildTrajectories(follower: Follower) {
        if (RED_ALLIANCE) {
            goalStartToCloseShoot = follower.pathBuilder()
                .addPath(BezierCurve(goalStartPos.mirror(), nearShootPos.mirror()))
                .setLinearHeadingInterpolation(goalStartPos.mirror().heading, nearShootPos.mirror().heading)
                .build()

            farStartToFarShoot = follower.pathBuilder()
                .addPath(BezierCurve(farStartPos.mirror(), farShootPos.mirror()))
                .setLinearHeadingInterpolation(farStartPos.mirror().heading, farShootPos.mirror().heading)
                .build()



            intakeSpike1 = follower.pathBuilder()
                .addPath(BezierCurve(spikeMark1PosPre.mirror(), spikeMark1PosPost.mirror()))
                .setLinearHeadingInterpolation(spikeMark1PosPre.mirror().heading, spikeMark1PosPost.mirror().heading)
                .build()

            intakeSpike2 = follower.pathBuilder()
                .addPath(BezierCurve(spikeMark2PosPre.mirror(), spikeMark2PosPost.mirror()))
                .setLinearHeadingInterpolation(spikeMark2PosPre.mirror().heading, spikeMark2PosPost.mirror().heading)
                .build()

            intakeSpike3 = follower.pathBuilder()
                .addPath(BezierCurve(spikeMark3PosPre.mirror(), spikeMark3PosPost.mirror()))
                .setLinearHeadingInterpolation(spikeMark3PosPre.mirror().heading, spikeMark3PosPost.mirror().heading)
                .build()

            intakeHumanPlayer = follower.pathBuilder()
                .addPath(BezierCurve(humanPlayerPosPre.mirror(), humanPlayerPosPost.mirror()))
                .setLinearHeadingInterpolation(humanPlayerPosPre.mirror().heading, humanPlayerPosPost.mirror().heading)
                .build()



            spike1ToDump = follower.pathBuilder()
                .addPath(BezierCurve(spikeMark1PosPost.mirror(), dumpBezierControlPoint.mirror(), dumpPose.mirror()))
                .setLinearHeadingInterpolation(spikeMark1PosPost.heading, dumpBezierControlPoint.heading)
                .build()

            spike2ToDump = follower.pathBuilder()
                .addPath(BezierCurve(spikeMark2PosPost.mirror(), dumpBezierControlPoint.mirror(), dumpPose.mirror()))
                .setLinearHeadingInterpolation(spikeMark2PosPost.mirror().heading, dumpBezierControlPoint.mirror().heading)
                .build()



            closeShootToSpike1 = follower.pathBuilder()
                .addPath(BezierCurve(nearShootPos.mirror(), spikeMark1PosPre.mirror()))
                .setLinearHeadingInterpolation(nearShootPos.mirror().heading, spikeMark1PosPre.mirror().heading)
                .build()

            closeShootToSpike2 = follower.pathBuilder()
                .addPath(BezierCurve(nearShootPos.mirror(), spikeMark2PosPre.mirror()))
                .setLinearHeadingInterpolation(nearShootPos.mirror().heading, spikeMark2PosPre.mirror().heading)
                .build()

            farShootToSpike3 = follower.pathBuilder()
                .addPath(BezierCurve(farShootPos.mirror(), spikeMark3PosPre.mirror()))
                .setLinearHeadingInterpolation(farShootPos.mirror().heading, spikeMark3PosPre.mirror().heading)
                .build()

            farShootToHumanPlayer = follower.pathBuilder()
                .addPath(BezierCurve(farShootPos.mirror(), humanPlayerPosPre.mirror()))
                .setLinearHeadingInterpolation(farShootPos.mirror().heading, humanPlayerPosPre.mirror().heading)
                .build()

            dumpToCloseShoot = follower.pathBuilder()
                .addPath(BezierCurve(dumpPose.mirror(), dumpBezierControlPoint.mirror(), nearShootPos.mirror()))
                .setLinearHeadingInterpolation(dumpPose.mirror().heading, nearShootPos.mirror().heading)
                .build()

            spike3ToFarShoot = follower.pathBuilder()
                .addPath(BezierCurve(spikeMark3PosPost.mirror(), farShootPos.mirror()))
                .setLinearHeadingInterpolation(spikeMark3PosPost.mirror().heading, farShootPos.mirror().heading)
                .build()

            humanPlayerToFarShoot = follower.pathBuilder()
                .addPath(BezierCurve(nearShootPos.mirror(), spikeMark2PosPre.mirror()))
                .setLinearHeadingInterpolation(nearShootPos.mirror().heading, spikeMark2PosPre.mirror().heading)
                .build()



            closeShootToClosePark = follower.pathBuilder()
                .addPath(BezierCurve(nearShootPos.mirror(), nearParkPos.mirror()))
                .setLinearHeadingInterpolation(nearShootPos.mirror().heading, nearParkPos.mirror().heading)
                .build()

            farShootToFarPark = follower.pathBuilder()
                .addPath(BezierCurve(farShootPos.mirror(), farParkPos.mirror()))
                .setLinearHeadingInterpolation(farShootPos.mirror().heading, farParkPos.mirror().heading)
                .build()
        } else {
            goalStartToCloseShoot = follower.pathBuilder()
                .addPath(BezierCurve(goalStartPos, nearShootPos))
                .setLinearHeadingInterpolation(goalStartPos.heading, nearShootPos.heading)
                .build()

            farStartToFarShoot = follower.pathBuilder()
                .addPath(BezierCurve(farStartPos, farShootPos))
                .setLinearHeadingInterpolation(farStartPos.heading, farShootPos.heading)
                .build()



            intakeSpike1 = follower.pathBuilder()
                .addPath(BezierCurve(spikeMark1PosPre, spikeMark1PosPost))
                .setLinearHeadingInterpolation(spikeMark1PosPre.heading, spikeMark1PosPost.heading)
                .build()

            intakeSpike2 = follower.pathBuilder()
                .addPath(BezierCurve(spikeMark2PosPre, spikeMark2PosPost))
                .setLinearHeadingInterpolation(spikeMark2PosPre.heading, spikeMark2PosPost.heading)
                .build()

            intakeSpike3 = follower.pathBuilder()
                .addPath(BezierCurve(spikeMark3PosPre, spikeMark3PosPost))
                .setLinearHeadingInterpolation(spikeMark3PosPre.heading, spikeMark3PosPost.heading)
                .build()

            intakeHumanPlayer = follower.pathBuilder()
                .addPath(BezierCurve(humanPlayerPosPre, humanPlayerPosPost))
                .setLinearHeadingInterpolation(humanPlayerPosPre.heading, humanPlayerPosPost.heading)
                .build()



            spike1ToDump = follower.pathBuilder()
                .addPath(BezierCurve(spikeMark1PosPost, dumpBezierControlPoint, dumpPose))
                .setLinearHeadingInterpolation(spikeMark1PosPost.heading, dumpBezierControlPoint.heading)
                .build()

            spike2ToDump = follower.pathBuilder()
                .addPath(BezierCurve(spikeMark2PosPost, dumpBezierControlPoint, dumpPose))
                .setLinearHeadingInterpolation(spikeMark2PosPost.heading, dumpBezierControlPoint.heading)
                .build()



            closeShootToSpike1 = follower.pathBuilder()
                .addPath(BezierCurve(nearShootPos, spikeMark1PosPre))
                .setLinearHeadingInterpolation(nearShootPos.heading, spikeMark1PosPre.heading)
                .build()

            closeShootToSpike2 = follower.pathBuilder()
                .addPath(BezierCurve(nearShootPos, spikeMark2PosPre))
                .setLinearHeadingInterpolation(nearShootPos.heading, spikeMark2PosPre.heading)
                .build()

            farShootToSpike3 = follower.pathBuilder()
                .addPath(BezierCurve(farShootPos, spikeMark3PosPre))
                .setLinearHeadingInterpolation(farShootPos.heading, spikeMark3PosPre.heading)
                .build()

            farShootToHumanPlayer = follower.pathBuilder()
                .addPath(BezierCurve(farShootPos, humanPlayerPosPre))
                .setLinearHeadingInterpolation(farShootPos.heading, humanPlayerPosPre.heading)
                .build()

            dumpToCloseShoot = follower.pathBuilder()
                .addPath(BezierCurve(dumpPose, dumpBezierControlPoint, nearShootPos))
                .setLinearHeadingInterpolation(dumpPose.heading, nearShootPos.heading)
                .build()

            spike3ToFarShoot = follower.pathBuilder()
                .addPath(BezierCurve(spikeMark3PosPost, farShootPos))
                .setLinearHeadingInterpolation(spikeMark3PosPost.heading, farShootPos.heading)
                .build()

            humanPlayerToFarShoot = follower.pathBuilder()
                .addPath(BezierCurve(nearShootPos, spikeMark2PosPre))
                .setLinearHeadingInterpolation(nearShootPos.heading, spikeMark2PosPre.heading)
                .build()



            closeShootToClosePark = follower.pathBuilder()
                .addPath(BezierCurve(nearShootPos, nearParkPos))
                .setLinearHeadingInterpolation(nearShootPos.heading, nearParkPos.heading)
                .build()

            farShootToFarPark = follower.pathBuilder()
                .addPath(BezierCurve(farShootPos, farParkPos))
                .setLinearHeadingInterpolation(farShootPos.heading, farParkPos.heading)
                .build()
        }
    }
    //endregion

}