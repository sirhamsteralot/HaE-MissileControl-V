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
            private const double dampingFactor = 0.2;
            private const double nominalSpeed = 80;
            private const double cancelGain = 3;
            private const double maxForwardAccelCapability = 80;
            private const double maxPredictTime = 15;
            private const double seekerLatency = 0.16;
            private const double interceptRadius = 12.5;
            private const double accelFilterAlpha = 0.8;
            private const double DeltaTime = 1.0 / 60.0;
            private const double nHeadingBiasMin = 0;
            private const double nHeadingBiasMax = 0.4;
            private const double maxG = 100.0;
            private const double MaxAccelAngleDegrees = 35;
            private readonly double CosMaxAngle = Math.Cos(MaxAccelAngleDegrees * Math.PI / 180f);

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
            private PIDController yawPID = new PIDController(18.0, 0.0, 0.3);
            private PIDController pitchPID = new PIDController(18.0, 0.0, 0.3);

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
                Vector3D forwardNorm = Velocity.LengthSquared() > 1e-6f
                    ? Vector3D.Normalize(Velocity)
                    : LOS;

                Vector3D relVel = targetVel - Velocity;

                // 2) Estimate & filter target accel
                Vector3D rawTargetAccel = (targetVel - prevTargetVel) / deltaTime;
                Vector3D targetAccel = Vector3D.Lerp(rawTargetAccel, prevTargetAccel, accelFilterAlpha);
                prevTargetAccel = targetAccel;
                prevTargetVel = targetVel;

                double speed = Math.Max(1f, Velocity.Length());
                double tti = range / speed;

                Vector3D predTgtPos = targetPos
                        + targetVel * tti
                        + 0.5f * targetAccel * tti * tti;
                Vector3D predLOS = Vector3D.Normalize(predTgtPos - Position);

                // 3) Compute aspect angle factor to avoid overshooting at high aspect
                Vector3D targetVelNorm = targetVel.LengthSquared() > 1e-6f
                    ? Vector3D.Normalize(targetVel)
                    : LOS;
                double aspectAng = Math.Acos(MathHelper.Clamp(Vector3D.Dot(LOS, targetVelNorm), -1f, 1f));
                // leadWeight goes from 0 (pure LOS at 90°) to 1 (max lead when aspect=0)
                double leadWeight = MathHelper.Clamp(Math.Cos(aspectAng), 0.9f, 1f);

                // 4) Guidance branch selection
                Vector3D rawAccel;
                const double closingSpeedThreshold = 1f;

                if (range > interceptRadius)
                {
                    // --- PN guidance with closing-speed check ---
                    double lookahead = Math.Min(maxPredictTime, range / Math.Max(Velocity.Length(), 1f));
                    lookahead += seekerLatency;

                    double cappedLookahead = Math.Min(lookahead, 2.0);
                    Vector3D predictedTargetVel = targetVel + targetAccel * cappedLookahead;

                    if (predictedTargetVel.Length() > 104)
                        predictedTargetVel = Vector3D.Normalize(predictedTargetVel) * 104;

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
                        double dynamicNavP = baseNavP * MathHelper.Clamp(closingSpeed / nominalSpeed, 0.5f, 2f);
                        Vector3D omega = Vector3D.Cross(futureLOS, relVel) / (range * range);
                        Vector3D desA = dynamicNavP * closingSpeed * Vector3D.Cross(omega, futureLOS);
                        Vector3D latA = desA - Vector3D.Dot(desA, futureLOS) * futureLOS;

                        double w = MathHelper.Clamp(1f / (range + 0.1f), nHeadingBiasMin, nHeadingBiasMax);
                        // incorporate aspect-based leadWeight to reduce over-leading at high aspect
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
                    double w = MathHelper.Clamp(1f / (range + 0.1f), nHeadingBiasMin, nHeadingBiasMax);
                    Vector3D baseDir = Vector3D.Normalize(
                        Vector3D.Lerp(LOS, predLOS, w)
                    );

                    // lateral cancel
                    Vector3D latVel = relVel - Vector3D.Dot(relVel, LOS) * LOS;
                    Vector3D cancel = latVel.LengthSquared() > 1e-6f
                        ? Vector3D.Normalize(latVel)
                        : Vector3D.Zero;

                    // blend cancel + baseDir, then apply aspect scaling
                    double cancelWeight = 0.3f * leadWeight;
                    double biasWeight = 1f - cancelWeight;
                    Vector3D blend = Vector3D.Normalize(cancelWeight * cancel + biasWeight * baseDir);

                    rawAccel = blend * maxForwardAccelCapability;
                }

                double closingVelocity = -Vector3D.Dot(relVel, LOS);
                Vector3D latRelVel = relVel * LOS;
                Vector3D cancelAccel = latRelVel * cancelGain;
                bool hyperAgressive = false;

                if (closingVelocity < -25 && rangeVec.Length() > 100)
                {
                    rawAccel += cancelAccel;
                    hyperAgressive = true;
                }

                // 5) Damping
                Vector3D damped = Vector3D.Lerp(prevAccel, rawAccel, dampingFactor);

                // 6) Jerk limiting
                Vector3D jerk = (damped - prevAccel) / deltaTime;
                double maxJerk = maxG * 9.81f / deltaTime;
                Vector3D clampedJerk = jerk;
                if (jerk.Length() > maxJerk)
                {
                    clampedJerk = Vector3D.Normalize(jerk) * maxJerk;
                }
                Vector3D finalAccel = prevAccel + clampedJerk * deltaTime;
                prevAccel = finalAccel;

                // 7) Angular boresight clamp
                Vector3D blendForward = Vector3D.Normalize(forwardNorm * 0.8f + predLOS * 0.2f);
                double maxAngleDeg = MathHelper.Lerp(90.0, 30.0, MathHelper.Clamp(range / 300.0, 0.0, 1.0));
                if (!hyperAgressive)
                    finalAccel = LimitAngleFromForward(finalAccel, blendForward, maxForwardAccelCapability, maxAngleDeg);
                else
                    finalAccel = cancelAccel * (LOS * 0.5 * maxForwardAccelCapability);

                // 8) G-limit
                double load = finalAccel.Length() / 9.81f;
                if (load > maxG)
                    finalAccel *= (maxG / load);

                return finalAccel;
            }

            private Vector3D LimitAngleFromForward(Vector3D accel, Vector3D forwardDir, double accelMagnitude, double maxAngleDeg)
            {
                Vector3D accelDir = Vector3D.Normalize(accel);
                double dot = Vector3D.Dot(accelDir, forwardDir);

                if (dot >= CosMaxAngle)
                    return accel; // Already within allowed cone

                // Clamp to cone edge
                Vector3D clampedDir = Vector3D.Normalize(Vector3D.Lerp(forwardDir, accelDir, 1f));
                double clampedDot = Vector3D.Dot(clampedDir, forwardDir);
                if (clampedDot < CosMaxAngle)
                {
                    // Clamp to exact cone edge
                    Vector3D axis = Vector3D.Cross(forwardDir, accelDir);
                    if (axis.LengthSquared() < 1e-6f)
                        return forwardDir * accelMagnitude;

                    axis = Vector3D.Normalize(axis);
                    QuaternionD rot = QuaternionD.CreateFromAxisAngle(axis, maxAngleDeg * Math.PI / 180.0);
                    clampedDir = Vector3D.Transform(forwardDir, rot);
                }

                return clampedDir * accelMagnitude;
            }

            private void UpdateRadarRefreshRate(double distanceToTargetSquared)
            {
                if (radarTrackingModule == null)
                    return;

                if (distanceToTargetSquared > 3000 * 3000)
                {
                    radarTrackingModule.ForcedRefreshRate = 50;
                }
                else if (distanceToTargetSquared > 1500 * 1500)
                {
                    radarTrackingModule.ForcedRefreshRate = 10;
                }
                else if (distanceToTargetSquared > 1000 * 1000)
                {
                    radarTrackingModule.ForcedRefreshRate = 5;
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
                double yaw, pitch;

                if (flightMovementBlock != null && !flightMovementBlock.Closed)
                {
                    GyroUtils.DirectionToPitchYaw(
                        flightMovementBlock.WorldMatrix.Forward,
                        flightMovementBlock.WorldMatrix.Left,
                        flightMovementBlock.WorldMatrix.Up,
                        aimDirection, out yaw, out pitch
                    );

                    double currentTime = currentPbTime * DeltaTime;

                    double yawOutput = yawPID.Compute(yaw, currentTime);
                    double pitchOutput = pitchPID.Compute(pitch, currentTime);

                    GyroUtils.ApplyGyroOverride(gyros, flightMovementBlock.WorldMatrix, pitchOutput, yawOutput, 0);
                }
                else if (thrustDirectionReference != null && thrustDirectionReference.IsFunctional)
                {
                    GyroUtils.DirectionToPitchYaw(
                        thrustDirectionReference.WorldMatrix.Forward,
                        thrustDirectionReference.WorldMatrix.Left,
                        thrustDirectionReference.WorldMatrix.Up,
                        aimDirection, out yaw, out pitch
                    );

                    double currentTime = currentPbTime * DeltaTime;

                    double yawOutput = yawPID.Compute(yaw, currentTime);
                    double pitchOutput = pitchPID.Compute(pitch, currentTime);

                    GyroUtils.ApplyGyroOverride(gyros, thrustDirectionReference.WorldMatrix, pitchOutput, yawOutput, 0);
                }
            }
        }
    }
}

