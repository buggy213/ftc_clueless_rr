package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.path.LineSegment
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.path.QuinticSplineSegment
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints
import com.acmerobotics.roadrunner.util.Angle

/**
 * Easy-to-use builder for creating [Trajectory] instances.
 *
 * @param startPose start pose
 * @param globalConstraints global drive constraints (overridable for specific segments)
 * @param resolution resolution used for path-based segments (see [PathTrajectorySegment])
 */
class TrajectoryBuilderExtended @JvmOverloads constructor(
        startPose: Pose2d,
        private val globalConstraints: DriveConstraints,
        private val resolution: Int = 250
) {
    private var currentPose: Pose2d = startPose
    private val trajectorySegments = mutableListOf<TrajectorySegment>()
    private var paths = mutableListOf<Path>()
    private var constraintsList = mutableListOf<TrajectoryConstraints>()
    private var composite = false
    private var reversed = false

    /**
     * Reverse the direction of robot travel.
     */
    fun reverse(): TrajectoryBuilderExtended {
        reversed = !reversed
        return this
    }

    /**
     * Sets the robot travel direction.
     */
    fun setReversed(reversed: Boolean): TrajectoryBuilderExtended {
        this.reversed = reversed
        return this
    }

    /**
     * Adds a point turn.
     *
     * @param angle angle to turn by (relative to the current heading)
     * @param constraintsOverride turn-specific drive constraints
     */
    // TODO: support turns that are greater than 180deg?
    @JvmOverloads
    fun turn(angle: Double, constraintsOverride: DriveConstraints? = null): TrajectoryBuilderExtended {
        return turnTo(Angle.norm(currentPose.heading + angle), constraintsOverride)
    }

    /**
     * Adds a point turn.
     *
     * @param heading heading to turn to
     * @param constraintsOverride turn-specific drive constraints
     */
    @JvmOverloads
    fun turnTo(heading: Double, constraintsOverride: DriveConstraints? = null): TrajectoryBuilderExtended {
        if (composite) {
            closeComposite()
        }

        val constraints = constraintsOverride ?: globalConstraints
        val pointTurn = PointTurn(currentPose, heading, constraints)
        trajectorySegments.add(pointTurn)
        currentPose = Pose2d(currentPose.pos(), heading)

        return this
    }

    /**
     * Adds a line path segment.
     *
     * @param pos end position
     * @param interpolator heading interpolator
     * @param constraintsOverride line-specific drive constraints
     */
    @JvmOverloads
    fun lineTo(pos: Vector2d, interpolator: HeadingInterpolator = TangentInterpolator(), constraintsOverride: TrajectoryConstraints? = null): TrajectoryBuilderExtended {
        val postBeginComposite = if (!interpolator.respectsDerivativeContinuity() && composite) {
            closeComposite()
            true
        } else {
            false
        }

        val constraints = constraintsOverride ?: globalConstraints
        val line = if (reversed) {
            Path(LineSegment(pos, currentPose.pos()), interpolator, true)
        } else {
            Path(LineSegment(currentPose.pos(), pos), interpolator, false)
        }
        if (composite) {
            paths.add(line)
            constraintsList.add(constraints)
        } else {
            trajectorySegments.add(PathTrajectorySegment(listOf(line), listOf(constraints), resolution))
        }
        currentPose = Pose2d(pos, currentPose.heading)

        if (postBeginComposite) {
            beginComposite()
        }

        return this
    }

    /**
     * Adds a strafe path segment.
     *
     * @param pos end position
     */
    fun strafeTo(pos: Vector2d) = lineTo(pos, ConstantInterpolator(currentPose.heading))

    /**
     * Adds a line straight forward.
     *
     * @param distance distance to travel forward
     */
    fun forward(distance: Double): TrajectoryBuilderExtended {
        return lineTo(currentPose.pos() + Vector2d(
                distance * Math.cos(currentPose.heading),
                distance * Math.sin(currentPose.heading)
        ))
    }

    /**
     * Adds a line straight backward.
     *
     * @param distance distance to travel backward
     */
    fun back(distance: Double): TrajectoryBuilderExtended {
        reverse()
        forward(-distance)
        reverse()
        return this
    }

    /**
     * Adds a segment that strafes left in the robot reference frame.
     *
     * @param distance distance to strafe left
     */
    fun strafeLeft(distance: Double): TrajectoryBuilderExtended {
        return strafeTo(currentPose.pos() + Vector2d(
                distance * Math.cos(currentPose.heading + Math.PI / 2),
                distance * Math.sin(currentPose.heading + Math.PI / 2)
        ))
    }

    /**
     * Adds a segment that strafes right in the robot reference frame.
     *
     * @param distance distance to strafe right
     */
    fun strafeRight(distance: Double): TrajectoryBuilderExtended {
        return strafeLeft(-distance)
    }

    /**
     * Adds a spline segment.
     *
     * @param pose end pose
     * @param interpolator heading interpolator
     * @param constraintsOverride spline-specific constraints
     */
    @JvmOverloads
    fun splineTo(pose: Pose2d, interpolator: HeadingInterpolator = TangentInterpolator(), constraintsOverride: TrajectoryConstraints? = null): TrajectoryBuilderExtended {
        val postBeginComposite = if (!interpolator.respectsDerivativeContinuity() && composite) {
            closeComposite()
            true
        } else {
            false
        }

        val constraints = constraintsOverride ?: this.globalConstraints
        val derivMag = (currentPose.pos() distanceTo pose.pos())
        val spline = if (reversed) {
            Path(
                    QuinticSplineSegment(
                            QuinticSplineSegment.Waypoint(pose.x, pose.y, derivMag * Math.cos(pose.heading), derivMag * Math.sin(pose.heading)),
                            QuinticSplineSegment.Waypoint(currentPose.x, currentPose.y, derivMag * Math.cos(currentPose.heading), derivMag * Math.sin(currentPose.heading))
                    ),
                    interpolator,
                    true
            )
        } else {
            Path(
                    QuinticSplineSegment(
                            QuinticSplineSegment.Waypoint(currentPose.x, currentPose.y, derivMag * Math.cos(currentPose.heading), derivMag * Math.sin(currentPose.heading)),
                            QuinticSplineSegment.Waypoint(pose.x, pose.y, derivMag * Math.cos(pose.heading), derivMag * Math.sin(pose.heading))
                    ),
                    interpolator,
                    false
            )
        }
        if (composite) {
            paths.add(spline)
            constraintsList.add(constraints)
        } else {
            trajectorySegments.add(PathTrajectorySegment(listOf(spline), listOf(constraints), resolution))
        }
        currentPose = pose

        if (postBeginComposite) {
            beginComposite()
        }

        return this
    }

    /**
     * Adds a wait segment.
     *
     * @param duration wait duration
     */
    fun waitFor(duration: Double): TrajectoryBuilderExtended {
        trajectorySegments.add(WaitSegment(currentPose, duration))
        return this
    }

    /**
     * Begins a composite path trajectory segment backed by a single continuous profile.
     */
    fun beginComposite(): TrajectoryBuilderExtended {
        composite = true
        return this
    }

    /**
     * Closes a composite path trajectory segment (see [beginComposite]).
     */
    fun closeComposite(): TrajectoryBuilderExtended {
        composite = false
        if (paths.isNotEmpty() && constraintsList.isNotEmpty()) {
            trajectorySegments.add(PathTrajectorySegment(paths, constraintsList, resolution))
            paths = mutableListOf()
            constraintsList = mutableListOf()
        }
        return this
    }

    /**
     * Constructs the [Trajectory] instance.
     */
    fun build(): Trajectory {
        if (composite) {
            closeComposite()
        }
        return Trajectory(trajectorySegments)
    }

    fun moveCartesianRelative(x: Double, y: Double) : TrajectoryBuilderExtended {
        val angle = Math.atan2(x, y)
        val distance = Math.sqrt(x * x + y * y)
        return movePolarRelative(distance, angle)
    }

    fun movePolarRelative(distance: Double, angle: Double) : TrajectoryBuilderExtended {
        return lineTo(currentPose.pos() + Vector2d(
                distance * Math.cos(currentPose.heading + angle),
                distance * Math.sin(currentPose.heading + angle)
        ), ConstantInterpolator(currentPose.heading))
    }
}