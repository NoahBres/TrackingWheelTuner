import java.lang.Math.*

data class Vector2d @JvmOverloads constructor(
    val x: Double = 0.0,
    val y: Double = 0.0
) {
    fun norm() = sqrt(x*x + y*y)

    fun angle() = atan2(y, x)

    operator fun plus(other: Vector2d) =
        Vector2d(x + other.x, y + other.y)

    operator fun minus(other: Vector2d) =
        Vector2d(x - other.x, y - other.y)

    operator fun times(scalar: Double) = Vector2d(scalar * x, scalar * y)

    operator fun div(scalar: Double) = Vector2d(x / scalar, y / scalar)

    operator fun unaryMinus() = Vector2d(-x, -y)

    infix fun dot(other: Vector2d) = x * other.x + y * other.y

    infix fun distTo(other: Vector2d) = (this - other).norm()

    infix fun projectOnto(other: Vector2d) = other * (this dot other) / (other dot other)

    fun rotated(angle: Double): Vector2d {
        val newX = x * cos(angle) - y * sin(angle)
        val newY = x * sin(angle) + y * cos(angle)
        return Vector2d(newX, newY)
    }

    infix fun epsilonEquals(other: Vector2d) =
        x epsilonEquals other.x && y epsilonEquals other.y

    override fun toString() = String.format("(%.3f, %.3f)", x, y)
}

operator fun Double.times(vector: Vector2d) = vector.times(this)

operator fun Double.div(vector: Vector2d) = vector.div(this)