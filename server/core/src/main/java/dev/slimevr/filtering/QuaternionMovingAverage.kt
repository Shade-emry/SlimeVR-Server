package dev.slimevr.filtering

import com.jme3.system.NanoTimer
import dev.slimevr.VRServer
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Quaternion.Companion.IDENTITY

// Constants for filtering parameters
private const val SMOOTH_MULTIPLIER = 42f
private const val SMOOTH_MIN = 11f
private const val PREDICT_MULTIPLIER = 15f
private const val PREDICT_MIN = 10f
private const val PREDICT_BUFFER_SIZE = 6

class QuaternionMovingAverage(
    val type: TrackerFilters,
    var amount: Float = 0f,
    initialRotation: Quaternion = IDENTITY,
) {
    private var filteredQuaternion = IDENTITY
    private var smoothingQuaternion = IDENTITY
    private var latestQuaternion = IDENTITY
    private var rotBuffer: CircularArrayList<Quaternion>? = null
    private val fpsTimer = if (VRServer.instanceInitialized) VRServer.instance.fpsTimer else NanoTimer()
    private var frameCounter = 0
    private var lastAmt = 0f

    init {
        require(amount >= 0f) { "Amount must be non-negative" }
        when (type) {
            TrackerFilters.SMOOTHING -> {
                // Lower smoothFactor = more smoothing
                smoothFactor = SMOOTH_MULTIPLIER * (1 - amount.coerceAtMost(1f)) + SMOOTH_MIN
                if (amount > 1) smoothFactor /= amount // Hack for high amounts
            }
            TrackerFilters.PREDICTION -> {
                // Higher predictFactor = more prediction
                predictFactor = PREDICT_MULTIPLIER * amount + PREDICT_MIN
                rotBuffer = CircularArrayList(PREDICT_BUFFER_SIZE)
            }
            else -> {} // No filtering
        }
        resetQuats(initialRotation)
    }

    @Synchronized
    fun update() {
        when (type) {
            TrackerFilters.PREDICTION -> updatePrediction()
            TrackerFilters.SMOOTHING -> updateSmoothing()
            else -> filteredQuaternion = latestQuaternion.twinNearest(smoothingQuaternion)
        }
    }

    private fun updatePrediction() {
        if (rotBuffer!!.size > 0) {
            var predictedQuat = latestQuaternion
            rotBuffer?.forEach { predictedQuat *= it }

            // Ensure consistent hemisphere before interpolation
            predictedQuat = ensureConsistentHemisphere(filteredQuaternion, predictedQuat)

            // Interpolate towards the predicted rotation
            val amt = predictFactor * fpsTimer.timePerFrame
            filteredQuaternion = filteredQuaternion.interpR(predictedQuat, amt)
        }
    }

    private fun updateSmoothing() {
        frameCounter++
        var amt = smoothFactor * frameCounter * fpsTimer.timePerFrame
        amt = amt.coerceIn(lastAmt, 1f)
        lastAmt = amt

        val targetQuaternion = ensureConsistentHemisphere(smoothingQuaternion, latestQuaternion)
        filteredQuaternion = smoothingQuaternion.interpR(targetQuaternion, amt)
    }

    @Synchronized
    fun addQuaternion(q: Quaternion) {
        when (type) {
            TrackerFilters.PREDICTION -> {
                if (rotBuffer!!.size == rotBuffer!!.capacity()) {
                    rotBuffer?.remove(0)
                }
                rotBuffer?.add(latestQuaternion.inv().times(q))
            }
            TrackerFilters.SMOOTHING -> {
                frameCounter = 0
                lastAmt = 0f
                smoothingQuaternion = filteredQuaternion
            }
            else -> smoothingQuaternion = filteredQuaternion
        }
        latestQuaternion = q
    }

    fun resetQuats(q: Quaternion) {
        filteredQuaternion = q
        smoothingQuaternion = q
        latestQuaternion = q
        rotBuffer?.clear()
    }

    private fun ensureConsistentHemisphere(reference: Quaternion, target: Quaternion): Quaternion {
        return if (reference.dot(target) < 0) -target else target
    }
}
