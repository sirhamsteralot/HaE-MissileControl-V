using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using SpaceEngineers.Game.Utils;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    public partial class Program : MyGridProgram
    {
        public class Missile
        {
            private const double baseNavP = 6;
            private const double dampingFactor = 0.5;
            private const double nominalSpeed = 80;
            private const double cancelGain = 3;
            private const double maxForwardAccelCapability = 80;
            private const double maxTurnRateRadPerSec = 6;
            private const double referenceTurnRate = 6.2;

            private const double maxPredictTime = 15;
            private const double seekerLatency = 0.16;
            private const double interceptRadius = 15;
            private const double accelFilterAlpha = 0.8;
            private const double DeltaTime = 1.0 / 60.0;
            private const double nHeadingBiasMin = 0.4;
            private const double nHeadingBiasMax = 0.8;
            private const double maxG = 100.0;

            private const double ExternalToRadarToleranceSquared = 2000 * 2000;

            public enum MissileHealth
            {
                Unknown,
                Healthy,
                Degraded,
                Dead,
            }

            public enum MissileFlightState
            {
                Unknown,
                Launching,
                Cruising,
                Terminal,
            }

            public List<IMyThrust> thrusters = new List<IMyThrust>();
            public List<IMyGyro> gyros = new List<IMyGyro>();
            public List<IMyWarhead> warheads = new List<IMyWarhead>();
            public List<IMyGasTank> gasTanks = new List<IMyGasTank>();
            public List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();

            public IMyFlightMovementBlock flightMovementBlock;
            public IMyOffensiveCombatBlock offensiveCombatBlock;

            public MissileFlightState FlightState { get; private set; }
            public MissileHealth Health { get; private set; }
            public long lifeTimeCounter { get; private set; }

            private long lastPositionVelocityUpdateTime = 0;
            public Vector3D Position { get; private set; }
            public Vector3D Velocity { get; private set; }
            public Vector3D Forward => GetForward();

            public DLBus.DLBusDetectedEntity ExternalTarget { get; private set; }

            private Vector3D planetCenterPos;
            private double planetGravity;
            private RadarTrackingModule radarTrackingModule;
            private Vector3D majorityThrustDirectionLocal;
            private IMyTerminalBlock thrustDirectionReference;

            private Vector3D prevTargetVel;
            private Vector3D prevTargetAccel;
            private Vector3D prevAccel;
            private PIDController yawPID = new PIDController(2.5, 0.0, 0.3);
            private PIDController pitchPID = new PIDController(2.5, 0.0, 0.3);

            public Missile()
            {
                FlightState = MissileFlightState.Unknown;
                Health = MissileHealth.Unknown;
            }

            public bool Initialize()
            {
                if (flightMovementBlock != null && offensiveCombatBlock != null)
                {
                    radarTrackingModule = new RadarTrackingModule(flightMovementBlock, offensiveCombatBlock);
                    radarTrackingModule.ForcedRefreshRate = 10;
                }

                lifeTimeCounter = 0;

                UpdateMissileHealth();
                majorityThrustDirectionLocal = GetMajorityThrustDirectionLocal(thrusters);

                if (Health == MissileHealth.Healthy || Health == MissileHealth.Degraded)
                {
                    FlightState = MissileFlightState.Launching;
                    return true;
                }

                return false;
            }

            public void UpdatePlanetValues(Vector3D center, double gravity)
            {
                planetCenterPos = center;
                planetGravity = gravity;
            }

            public void Flight(long currentPbTime)
            {
                if (radarTrackingModule != null)
                    radarTrackingModule.UpdateTracking(currentPbTime);

                UpdateVelocityocityPosition(currentPbTime);

                if (lifeTimeCounter % 60 == 0)
                {
                    UpdateMissileHealth();
                }

                switch (FlightState)
                {
                    case MissileFlightState.Launching:
                        FlightLaunching(currentPbTime);
                        break;

                    case MissileFlightState.Cruising:
                        FlightCruising(currentPbTime);
                        break;

                    case MissileFlightState.Terminal:
                        FlightTerminal(currentPbTime);
                        break;

                    default:
                        throw new Exception("missile flight state unknown or other; this should never be reached");
                }

                lifeTimeCounter++;
            }

            public void UpdateTargetedEntity(DLBus.DLBusDetectedEntity detectedEntity)
            {
                ExternalTarget = detectedEntity;
            }

            private void FlightLaunching(long currentPbTime)
            {
                if (lifeTimeCounter < 60 * 1)
                {
                    AimInDirection(Forward, currentPbTime);
                    ThrustUtils.SetThrustBasedDot(thrusters, Forward);
                }
                else if (lifeTimeCounter < 60 * 5 && planetGravity != 0)
                {
                    long localCounter = lifeTimeCounter - 60 * 1;

                    double ratio = (double)localCounter / (60.0 * 4);

                    Vector3D planetDirection = Vector3D.Normalize(Position - planetCenterPos);

                    Vector3D aimVector = Vector3D.Normalize(Forward + (ratio * planetDirection));

                    AimInDirection(aimVector, currentPbTime);
                }
                else
                {
                    FlightState = MissileFlightState.Cruising;
                }
            }

            private void FlightCruising(long currentPbTime)
            {
                if (ExternalTarget != null)
                {
                    Vector3D predictedExternalPosition = ExternalTarget.LastKnownLocation + (ExternalTarget.LastKnownVelocity / 60);
                    Vector3D distanceFromTarget = predictedExternalPosition - Position;
                    UpdateRadarRefreshRate(distanceFromTarget.LengthSquared());

                    Vector3D directionToTarget = Vector3D.Normalize(distanceFromTarget);
                    AimInDirection(directionToTarget, currentPbTime);
                    ThrustUtils.SetThrustBasedDot(thrusters, directionToTarget);
                }
                else
                {
                    Vector3D fallbackDirection = Vector3D.Normalize(Velocity);
                    ThrustUtils.SetThrustBasedDot(thrusters, fallbackDirection);
                    AimInDirection(fallbackDirection, currentPbTime);
                }

                if (lifeTimeCounter > 0)
                {
                    FlightState = MissileFlightState.Terminal;
                }
            }

            private void FlightTerminal(long currentPbTime)
            {
                if (radarTrackingModule == null || !radarTrackingModule.IsTracking)
                {
                    // No target, fly straight
                    Vector3D fallbackDirection = Vector3D.Normalize(Velocity);
                    ThrustUtils.SetThrustBasedDot(thrusters, fallbackDirection);
                    AimInDirection(fallbackDirection, currentPbTime);
                    return;
                }

                Vector3D targetPos = radarTrackingModule.TargetPosition;
                Vector3D targetVel = radarTrackingModule.TargetVelocity;

                UpdateRadarRefreshRate(Vector3D.Distance(targetPos, Position));

                Vector3D gravitydirection = planetCenterPos - Position;
                double altitude = gravitydirection.Normalize();


                Vector3D accelCommand = ComputeGuidanceAccel(targetPos, targetVel, DeltaTime) - gravitydirection * planetGravity;

                if (Vector3D.IsZero(accelCommand))
                {
                    // Fallback to current velocity direction if guidance fails
                    Vector3D fallbackDirection = Vector3D.Normalize(Velocity);
                    ThrustUtils.SetThrustBasedDot(thrusters, fallbackDirection);
                    AimInDirection(fallbackDirection, currentPbTime);
                    return;
                }

                ThrustUtils.SetThrustBasedDot(thrusters, Vector3D.Normalize(accelCommand)); // Changed from requiredAccelDir
                AimInDirection(Vector3D.Normalize(accelCommand), currentPbTime); // Changed from requiredAccelDir
            }

            private Vector3D ComputeGuidanceAccel(
                Vector3D targetPos,
                Vector3D targetVel,
                double deltaTime
            )
            {
                // 1) Basic geometry
                Vector3D rangeVec = targetPos - Position;
                double range = rangeVec.Length();
                Vector3D LOS = Vector3D.Normalize(rangeVec);

                // Normalize forward for boresight usage
                Vector3D forwardNorm = Velocity.LengthSquared() > 1e-6
                    ? Vector3D.Normalize(Velocity)
                    : LOS;

                Vector3D relVel = targetVel - Velocity;

                // 2) Estimate & filter target accel
                Vector3D rawTargetAccel = (targetVel - prevTargetVel) / deltaTime;
                Vector3D targetAccel = Vector3D.Lerp(rawTargetAccel, prevTargetAccel, accelFilterAlpha);
                prevTargetAccel = targetAccel;
                prevTargetVel = targetVel;

                double speed = Math.Max(1.0, Velocity.Length());
                double tti = range / speed;
                double ttiCapped = Math.Min(2.5, tti);

                Vector3D predTgtPos = targetPos + targetVel * tti + 0.5 * targetAccel * ttiCapped * ttiCapped;
                Vector3D predLOS = Vector3D.Normalize(predTgtPos - Position);

                // 3) Compute aspect angle factor to avoid overshooting at high aspect
                Vector3D targetVelNorm = targetVel.LengthSquared() > 1e-6
                    ? Vector3D.Normalize(targetVel)
                    : LOS;
                double aspectAng = Math.Acos(MathHelper.Clamp(Vector3D.Dot(LOS, targetVelNorm), -1.0, 1.0));
                // leadWeight goes from 0 (pure LOS at 90°) to 1 (max lead when aspect=0)
                // reduced range to bias more toward LOS
                double leadWeight = MathHelper.Clamp(Math.Cos(aspectAng), 0.7, 0.9);

                // 4) Guidance branch selection
                Vector3D rawAccel;
                const double closingSpeedThreshold = 1.0;

                if (range > interceptRadius)
                {
                    // --- PN guidance with closing-speed check ---
                    double lookahead = Math.Min(maxPredictTime, range / Math.Max(Velocity.Length(), 1.0));
                    lookahead += seekerLatency;

                    double cappedLookahead = Math.Min(lookahead, 2.0);
                    Vector3D predictedTargetVel = targetVel + targetAccel * cappedLookahead;
                    if (predictedTargetVel.Length() > 104.0)
                        predictedTargetVel = Vector3D.Normalize(predictedTargetVel) * 104.0;

                    Vector3D futureTgtPos = targetPos + predictedTargetVel * lookahead;
                    Vector3D futureLOS = Vector3D.Normalize(futureTgtPos - Position);

                    double closingSpeed = -Vector3D.Dot(relVel, futureLOS);

                    if (closingSpeed < closingSpeedThreshold)
                    {
                        // fallback to pure LOS pursuit
                        rawAccel = futureLOS * maxForwardAccelCapability;
                    }
                    else
                    {
                        // scale PN gain by turn authority factor
                        double turnAuth = MathHelper.Clamp(maxTurnRateRadPerSec / referenceTurnRate, 0.1, 1.0);
                        double dynamicNavP = baseNavP * turnAuth * MathHelper.Clamp(closingSpeed / nominalSpeed, 0.5, 2.0);

                        Vector3D omega = Vector3D.Cross(futureLOS, relVel) / (range * range);
                        Vector3D desA = dynamicNavP * closingSpeed * Vector3D.Cross(omega, futureLOS);
                        Vector3D latA = desA - Vector3D.Dot(desA, futureLOS) * futureLOS;

                        double w = MathHelper.Clamp(
                            1.0 / (range + 0.1) * (1 + (1 - turnAuth)),
                            nHeadingBiasMin,
                            nHeadingBiasMax
                        );
                        // incorporate reduced leadWeight
                        Vector3D leadDir = Vector3D.Normalize(latA);
                        Vector3D blendDir = Vector3D.Normalize(
                            Vector3D.Lerp(futureLOS, leadDir, leadWeight)
                        );
                        Vector3D biasDir = Vector3D.Normalize(
                            Vector3D.Lerp(blendDir, futureLOS, w)
                        );

                        rawAccel = biasDir * maxForwardAccelCapability;
                    }
                }
                else
                {
                    // --- Terminal homing ---
                    double w = MathHelper.Clamp(1.0 / (range + 0.1), nHeadingBiasMin, nHeadingBiasMax);
                    Vector3D baseDir = Vector3D.Normalize(
                        Vector3D.Lerp(LOS, predLOS, w)
                    );

                    // lateral cancel
                    Vector3D latVel = relVel - Vector3D.Dot(relVel, LOS) * LOS;
                    Vector3D cancel = latVel.LengthSquared() > 1e-6
                        ? Vector3D.Normalize(latVel)
                        : Vector3D.Zero;

                    // blend cancel + baseDir then aspect scaling
                    double cancelWeight = 0.3 * leadWeight;
                    double biasWeight = 1.0 - cancelWeight;
                    Vector3D blend = Vector3D.Normalize(cancelWeight * cancel + biasWeight * baseDir);

                    rawAccel = blend * maxForwardAccelCapability;
                }

                double closingVelocity = -Vector3D.Dot(relVel, LOS);
                Vector3D latRelVel = relVel * LOS;
                Vector3D cancelAccel = latRelVel * cancelGain;
                bool hyperAgressive = false;
                if (closingVelocity < -25.0 && rangeVec.Length() > 100.0)
                {
                    rawAccel += cancelAccel;
                    hyperAgressive = true;
                }

                // 5) Damping
                Vector3D damped = Vector3D.Lerp(prevAccel, rawAccel, dampingFactor);

                // 6) Jerk limiting
                Vector3D jerk = (damped - prevAccel) / deltaTime;
                double maxJerk = maxG * 9.81 / deltaTime;
                Vector3D clampedJerk = jerk.Length() > maxJerk
                    ? Vector3D.Normalize(jerk) * maxJerk
                    : jerk;
                Vector3D accelCandidate = prevAccel + clampedJerk * deltaTime;

                // 7) Angular boresight clamp
                Vector3D blendForward = Vector3D.Normalize(forwardNorm * 0.4 + predLOS * 0.6);
                double maxAngleDeg = MathHelper.Lerp(50.0, 20.0, MathHelper.Clamp(range / 2000.0, 0.0, 1.0));
                Vector3D clippedAccel = !hyperAgressive
                    ? LimitAngleFromForward(accelCandidate, blendForward, maxForwardAccelCapability, maxAngleDeg)
                    : cancelAccel * (LOS * 0.5 * maxForwardAccelCapability);

                // 8) Explicit turn-rate limit
                Vector3D desiredDir = Vector3D.Normalize(clippedAccel);
                Vector3D prevDir = prevAccel.LengthSquared() > 1e-6
                    ? Vector3D.Normalize(prevAccel)
                    : forwardNorm;
                double maxDTheta = maxTurnRateRadPerSec * deltaTime;
                double cosAng = MathHelper.Clamp(Vector3D.Dot(prevDir, desiredDir), -1.0, 1.0);
                double ang = Math.Acos(cosAng);
                Vector3D newDir = ang <= maxDTheta
                    ? desiredDir
                    : Vector3D.Normalize(Slerp(prevDir, desiredDir, maxDTheta / ang));
                Vector3D finalAccel = newDir * clippedAccel.Length();

                prevAccel = finalAccel;
                return finalAccel;
            }

            private Vector3D Slerp(Vector3D a, Vector3D b, double t)
            {
                // Clamp t to [0,1]
                t = MathHelper.Clamp(t, 0.0, 1.0);

                double dot = Vector3D.Dot(a, b);
                // Clamp dot to avoid NaNs from Acos
                dot = MathHelper.Clamp(dot, -1.0, 1.0);

                double theta = Math.Acos(dot) * t;  // angle between a and result

                // Compute the orthonormal basis
                Vector3D relative = b - a * dot;
                if (relative.LengthSquared() < 1e-12)
                {
                    // a and b are nearly parallel: fallback to Lerp
                    return Vector3D.Normalize(Vector3D.Lerp(a, b, t));
                }
                Vector3D relativeNorm = Vector3D.Normalize(relative);

                // result = a*cos(theta) + relativeNorm*sin(theta)
                return a * Math.Cos(theta) + relativeNorm * Math.Sin(theta);
            }

            private Vector3D LimitAngleFromForward(Vector3D accel, Vector3D forwardDir, double accelMagnitude, double maxAngleDeg)
            {
                Vector3D accelDir = Vector3D.Normalize(accel);
                double dot = Vector3D.Dot(accelDir, forwardDir);

                double cosMaxAngle = Math.Cos(Math.PI * maxAngleDeg / 180);

                if (dot >= cosMaxAngle)
                    return accel; // Already within allowed cone

                // Clamp to cone edge
                Vector3D clampedDir = Vector3D.Normalize(Vector3D.Lerp(forwardDir, accelDir, 1f));
                double clampedDot = Vector3D.Dot(clampedDir, forwardDir);
                if (clampedDot < cosMaxAngle)
                {
                    Vector3D axis = Vector3D.Cross(forwardDir, accelDir);
                    if (axis.LengthSquared() < 1e-6f)
                        return forwardDir * accelMagnitude;

                    axis = Vector3D.Normalize(axis);
                    double angleRad = maxAngleDeg * Math.PI / 180f;

                    // Rodrigues' rotation formula
                    clampedDir = Vector3D.Normalize(
                        forwardDir * Math.Cos(angleRad)
                        + Vector3D.Cross(axis, forwardDir) * Math.Sin(angleRad)
                        + axis * Vector3D.Dot(axis, forwardDir) * (1 - Math.Cos(angleRad))
                    );
                }

                return clampedDir * accelMagnitude;
            }

            private void UpdateRadarRefreshRate(double distanceToTargetSquared)
            {
                if (radarTrackingModule == null)
                    return;

                if (distanceToTargetSquared > 3000 * 3000)
                {
                    radarTrackingModule.ForcedRefreshRate = 15;
                }
                else if (distanceToTargetSquared > 1500 * 1500)
                {
                    radarTrackingModule.ForcedRefreshRate = 5;
                }
                else if (distanceToTargetSquared > 1000 * 1000)
                {
                    radarTrackingModule.ForcedRefreshRate = 1;
                }
                else
                {
                    radarTrackingModule.ForcedRefreshRate = 1;
                }
            }

            private void UpdateMissileHealth()
            {
                if (radarTrackingModule == null)
                {
                    Health = MissileHealth.Degraded;
                    return;
                }

                if (!radarTrackingModule.CheckWorking())
                {
                    Health = MissileHealth.Degraded;
                    return;
                }

                int thrusterWorkingCount = thrusters.Count(x => x.IsWorking);

                if (thrusterWorkingCount != thrusters.Count)
                {
                    if (thrusterWorkingCount == 0)
                    {
                        Health = MissileHealth.Dead;
                        return;
                    }

                    Health = MissileHealth.Degraded;
                    return;
                }

                int gyroWorkingCount = gyros.Count(x => x.IsWorking);
                if (gyroWorkingCount != gyros.Count)
                {
                    if (gyroWorkingCount == 0)
                    {
                        Health = MissileHealth.Dead;
                        return;
                    }

                    Health = MissileHealth.Degraded;
                    return;
                }

                int warheadWorkingCount = warheads.Count(x => x.IsWorking);

                if (warheadWorkingCount != warheads.Count)
                {
                    if (warheadWorkingCount == 0)
                    {
                        Health = MissileHealth.Dead;
                        return;
                    }

                    Health = MissileHealth.Degraded;
                    return;
                }

                int gasTankWorkingCount = gasTanks.Count(x => x.IsWorking);

                if (gasTankWorkingCount != gasTanks.Count)
                {
                    if (gasTankWorkingCount == 0)
                    {
                        Health = MissileHealth.Dead;
                        return;
                    }

                    Health = MissileHealth.Degraded;
                    return;
                }

                int batteryWorkingCount = batteries.Count(x => x.IsWorking);

                if (batteryWorkingCount != batteries.Count)
                {
                    if (batteryWorkingCount == 0)
                    {
                        Health = MissileHealth.Dead;
                        return;
                    }

                    Health = MissileHealth.Degraded;
                    return;
                }

                Health = MissileHealth.Healthy;
            }

            private void UpdateVelocityocityPosition(long currentPbTime)
            {
                Vector3D currentPosition = Vector3D.Zero;

                foreach (var gyro in gyros)
                {
                    if (gyro.IsFunctional)
                    {
                        currentPosition = gyro.GetPosition();
                    }
                }

                long timeDifference = currentPbTime - lastPositionVelocityUpdateTime;

                Vector3D velocity = (currentPosition - Position) / timeDifference * 60;

                Position = currentPosition;
                Velocity = velocity;
                lastPositionVelocityUpdateTime = currentPbTime;
            }

            private Vector3D GetForward()
            {
                if (flightMovementBlock != null && !flightMovementBlock.Closed)
                {
                    return flightMovementBlock.WorldMatrix.Forward;
                }
                else if (thrustDirectionReference != null)
                {
                    return VectorUtils.TransformDirLocalToWorld(thrustDirectionReference.WorldMatrix, majorityThrustDirectionLocal);
                }
                else
                {
                    return Vector3D.Zero;
                }
            }

            private Vector3D GetMajorityThrustDirectionLocal(List<IMyThrust> thrusters)
            {
                var directionCounts = new Dictionary<Vector3D, int>();


                foreach (var thruster in thrusters)
                {
                    if (thrustDirectionReference == null && !thruster.Closed)
                    {
                        thrustDirectionReference = thruster;
                    }

                    var relativeThrustVector = VectorUtils.TransformDirWorldToLocal(thrustDirectionReference.WorldMatrix, thruster.WorldMatrix.Backward);

                    if (directionCounts.ContainsKey(relativeThrustVector))
                    {
                        directionCounts[relativeThrustVector]++;
                    }
                    else
                    {
                        directionCounts[relativeThrustVector] = 1;
                    }
                }

                Vector3D majorityDirection = Vector3D.Zero;
                int maxCount = 0;

                foreach (var kvp in directionCounts)
                {
                    if (kvp.Value > maxCount)
                    {
                        maxCount = kvp.Value;
                        majorityDirection = kvp.Key;
                    }
                }

                return majorityDirection;
            }


            private void AimInDirection(Vector3D aimDirection, long currentPbTime)
            {
                // ——————————————————————————————————————————————
                // 1) Ensure we have our only reference block
                // ——————————————————————————————————————————————
                if (thrustDirectionReference == null || !thrustDirectionReference.IsFunctional)
                    return;
                var refMatrix = thrustDirectionReference.WorldMatrix;
                
                // programmable-block time (ticks) → seconds
                double t = currentPbTime * DeltaTime;

                    // ——————————————————————————————————————————————
                    // 2) Compute raw yaw & pitch error
                    // ——————————————————————————————————————————————
                    double yawError, pitchError;
                DirectionToPitchYaw(
                    refMatrix.Forward,
                    refMatrix.Right,      // ← use Right for correct yaw sign
                    refMatrix.Up,
                    aimDirection,
                    out yawError,
                    out pitchError
                );

                // ——————————————————————————————————————————————
                // 3) PID loops
                // ——————————————————————————————————————————————
                double rawYaw   = yawPID  .Compute(yawError,   t);
                double rawPitch = pitchPID.Compute(pitchError, t);

                // ——————————————————————————————————————————————
                // 4) Clamp outputs to ±1
                // ——————————————————————————————————————————————
                const double maxGyro = 1.0;
                double yawOut   = MathHelper.Clamp(rawYaw,   -maxGyro, maxGyro);
                double pitchOut = MathHelper.Clamp(rawPitch, -maxGyro, maxGyro);

                // ——————————————————————————————————————————————
                // 5) Dead-zone: if both errors are tiny, stop overriding
                // ——————————————————————————————————————————————
                const double angleThreshold = 0.002; // ~0.1°
                if (Math.Abs(yawError) < angleThreshold && Math.Abs(pitchError) < angleThreshold)
                {
                    foreach (var g in gyros)
                        g.GyroOverride = false;
                    return;
                }

                // ——————————————————————————————————————————————
                // 6) Send commands to all gyros
                // ——————————————————————————————————————————————
                ApplyGyroOverride(gyros, refMatrix, pitchOut, yawOut, 0);
            }

            public static void ApplyGyroOverride(
                List<IMyGyro> gyros,
                MatrixD reference,
                double pitch, double yaw, double roll,
                bool onlyUpdateOne = false
            )
            {
                // local → world
                var local = new Vector3D(-pitch, yaw, roll);
                var world = Vector3D.TransformNormal(local, reference);

                if (!world.IsValid()) return;

                foreach (var gyro in gyros)
                {
                    // world → gyro-local
                    var rotor = Vector3D.TransformNormal(world, Matrix.Transpose(gyro.WorldMatrix));

                    gyro.Pitch = (float)rotor.X;
                    gyro.Yaw = (float)rotor.Y;
                    gyro.Roll = (float)rotor.Z;
                    gyro.GyroOverride = true;

                    if (onlyUpdateOne) break;
                }
            }

            public static void DirectionToPitchYaw(
                Vector3D forward,   // ship’s forward
                Vector3D right,     // ship’s right (not left!)
                Vector3D up,        // ship’s up
                Vector3D targetDir, // normalized aim direction
                out double yaw, 
                out double pitch
            ) {
                // 1) split target into “up” vs “horizontal” parts
                var upComp    = VectorUtils.Project(targetDir, up);
                var horizComp = targetDir - upComp;

                // 2) angles between forward/horizontal and full/ horizontal
                yaw   = VectorUtils.GetAngle(forward,   horizComp);
                pitch = VectorUtils.GetAngle(targetDir, horizComp);

                // 3) sign‐bit: if target is to your right, positive yaw; to your left, negative
                yaw   *= -Math.Sign(right.Dot(targetDir));
                //    if target is above your plane it's positive pitch, else negative
                pitch *= -Math.Sign(up   .Dot(targetDir));

                // 4) exactly behind you?
                if (yaw == 0 && pitch == 0 && targetDir.Dot(forward) < 0)
                    yaw = Math.PI;
            }

        }
    }
}

