import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils

fun main(args: Array<String>) {
    val LATERAL_DISTANCE = 10.0
    val FORWARD_OFFSET = 4.0

    val wheelPoses = listOf(
        Pose2d(0.0, LATERAL_DISTANCE / 2, 0.0),
        Pose2d(0.0, -LATERAL_DISTANCE / 2, 0.0),
        Pose2d(FORWARD_OFFSET, 0.0, Math.toRadians(90.0))
    )

    val inverseMatrix = Array2DRowRealMatrix(3, 3)
    for (i in 0..2) {
        val orientationVector = wheelPoses[i].headingVec()
        val positionVector = wheelPoses[i].vec()
        inverseMatrix.setEntry(i, 0, orientationVector.x)
        inverseMatrix.setEntry(i, 1, orientationVector.y)
        inverseMatrix.setEntry(i, 2, positionVector.x * orientationVector.y - positionVector.y * orientationVector.x)
    }

    val forwardSolver = LUDecomposition(inverseMatrix).solver

    val wheelPositions = getWheelPositions(0, 0, 0)
    val lastWheelPositions = getWheelPositions(0, 0, 0)

    val wheelDeltas = wheelPositions.zip(lastWheelPositions).map { it.first - it.second }
    val rawPoseDelta =
        forwardSolver.solve(MatrixUtils.createRealMatrix(arrayOf(wheelDeltas.toDoubleArray())).transpose())
    val robotPoseDelta = Pose2d(
        rawPoseDelta.getEntry(0, 0),
        rawPoseDelta.getEntry(1, 0),
        rawPoseDelta.getEntry(2, 0)
    )

}

fun getWheelPositions(left: Int, right: Int, front: Int): List<Double> {

    return listOf(
        encoderTicksToInches(left),
        encoderTicksToInches(right),
        encoderTicksToInches(front)
    )
}

fun encoderTicksToInches(ticks: Int): Double {
    val WHEEL_RADIUS = 1.9685;
    val GEAR_RATIO = 0.5;
    val TICKS_PER_REV = 386.3

    return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks.toDouble() / TICKS_PER_REV
}