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

        private Vector3D ComputeGuidanceAccel(Vector3D targetPos, Vector3D targetVel, double deltaTime)
        {
            // 1) Geometry & relative state
            var rangeVec      = targetPos - Position;
            var range         = rangeVec.Length();
            var LOS           = rangeVec / range; // normalized
            var forwardNorm   = Velocity.LengthSquared() > 1e-6 
                                ? Vector3D.Normalize(Velocity) 
                                : LOS;
            var relVel        = targetVel - Velocity;
            var speed         = Math.Max(1.0, Velocity.Length());

            // 2) Target acceleration estimation (filtered)
            var targetAccel = EstimateTargetAccel(targetVel, deltaTime);

            // 3) Time-to-impact and predicted LOS
            var timeToImpact = range / speed;
            var predLOS      = PredictLOS(targetPos, targetVel, targetAccel, timeToImpact);

            // 4) Aspect‐based lead weight
            var leadWeight   = ComputeLeadWeight(LOS, targetVel);

            // 5) Choose guidance law
            Vector3D rawAccel;
            if (range > interceptRadius)
                rawAccel = ComputePNGuidance(range, LOS, relVel, targetPos, targetVel, targetAccel, leadWeight);
            else
                rawAccel = ComputeTerminalHoming(range, LOS, predLOS, relVel, leadWeight);

            // 6) Closing‐velocity tweak
            rawAccel = ApplyClosingVelocityCancel(rawAccel, rangeVec, relVel, ref rawAccel);

            // 7) Damping & jerk‐limit
            var dampedAccel = Vector3D.Lerp(prevAccel, rawAccel, dampingFactor);
            var jerkLimited = LimitJerk(dampedAccel, deltaTime);

            // 8) Boresight & G‐limit
            var boresighted = ApplyBoresightClamp(jerkLimited, forwardNorm, predLOS, range, ref rawAccel);
            return ApplyGLimit(boresighted);
        }

        #region — Helpers —

        private Vector3D EstimateTargetAccel(Vector3D targetVel, double dt)
        {
            var raw = (targetVel - prevTargetVel) / dt;
            var filt = Vector3D.Lerp(raw, prevTargetAccel, accelFilterAlpha);
            prevTargetVel   = targetVel;
            prevTargetAccel = filt;
            return filt;
        }

        private Vector3D PredictLOS(Vector3D pos, Vector3D vel, Vector3D accel, double t)
        {
            var predPos = pos + vel * t + 0.5 * accel * t * t;
            return Vector3D.Normalize(predPos - Position);
        }

        private double ComputeLeadWeight(Vector3D LOS, Vector3D targetVel)
        {
            var tgtDir = targetVel.LengthSquared() > 1e-6 
                        ? Vector3D.Normalize(targetVel) 
                        : LOS;
            var cosA   = MathHelper.Clamp(Vector3D.Dot(LOS, tgtDir), -1, 1);
            // weight in [0.9,1.0] to avoid zero‐lead at high aspect
            return MathHelper.Clamp(cosA, 0.9, 1.0);
        }

        private Vector3D ComputePNGuidance(
            double range, Vector3D LOS, Vector3D relVel,
            Vector3D tgtPos, Vector3D tgtVel, Vector3D tgtAccel, double leadWeight)
        {
            // Predict target velocity & cap
            var lookahead = Math.Min(maxPredictTime, range / Math.Max(Velocity.Length(), 1.0))
                            + seekerLatency;
            var capped = Math.Min(lookahead, 2.0);
            var predVel = tgtVel + tgtAccel * capped;
            if (predVel.Length() > 104) 
                predVel = Vector3D.Normalize(predVel) * 104;

            // Future LOS & closing speed
            var futureLOS     = Vector3D.Normalize((tgtPos + predVel * lookahead) - Position);
            var closingSpeed  = -Vector3D.Dot(relVel, futureLOS);
            const double CLOSING_THRESHOLD = 1.0;

            if (closingSpeed < CLOSING_THRESHOLD)
                return futureLOS * maxForwardAccelCapability;

            // Proportional navigation
            var navP   = baseNavP * MathHelper.Clamp(closingSpeed / nominalSpeed, 0.5, 2.0);
            var omega  = Vector3D.Cross(futureLOS, relVel) / (range * range);
            var desA   = navP * closingSpeed * Vector3D.Cross(omega, futureLOS);
            var latA   = desA - Vector3D.Dot(desA, futureLOS) * futureLOS;

            // Blend LOS & lead, then bias
            var w       = MathHelper.Clamp(1.0 / (range + 0.1), nHeadingBiasMin, nHeadingBiasMax);
            var leadDir = Vector3D.Normalize(latA);
            var blend   = Vector3D.Normalize(Vector3D.Lerp(futureLOS, leadDir, leadWeight));
            var bias    = Vector3D.Normalize(Vector3D.Lerp(blend, futureLOS, w));

            return bias * maxForwardAccelCapability;
        }

        private Vector3D ComputeTerminalHoming(
            double range, Vector3D LOS, Vector3D predLOS, Vector3D relVel, double leadWeight)
        {
            var w        = MathHelper.Clamp(1.0 / (range + 0.1), nHeadingBiasMin, nHeadingBiasMax);
            var baseDir  = Vector3D.Normalize(Vector3D.Lerp(LOS, predLOS, w));

            // lateral velocity cancellation
            var latVel   = relVel - Vector3D.Dot(relVel, LOS) * LOS;
            var cancel   = latVel.LengthSquared() > 1e-6 
                        ? Vector3D.Normalize(latVel) 
                        : Vector3D.Zero;
            var cancelW  = 0.3 * leadWeight;
            var blendDir = Vector3D.Normalize(cancel * cancelW + baseDir * (1 - cancelW));

            return blendDir * maxForwardAccelCapability;
        }

        private Vector3D ApplyClosingVelocityCancel(
            Vector3D accel, Vector3D rangeVec, Vector3D relVel, ref Vector3D rawAccel)
        {
            var closingVel = -Vector3D.Dot(relVel, rangeVec / rangeVec.Length());
            if (closingVel < -25 && rangeVec.Length() > 100)
            {
                var cancelAccel = (relVel * (rangeVec / rangeVec.Length())) * cancelGain;
                rawAccel += cancelAccel;
            }
            return rawAccel;
        }

        private Vector3D LimitJerk(Vector3D damped, double dt)
        {
            var jerk     = (damped - prevAccel) / dt;
            var maxJ     = maxG * 9.81 / dt;
            var clamped  = jerk.Length() > maxJ 
                        ? Vector3D.Normalize(jerk) * maxJ 
                        : jerk;
            prevAccel    = prevAccel + clamped * dt;
            return prevAccel;
        }

        private Vector3D ApplyBoresightClamp(
            Vector3D accel, Vector3D forwardNorm, Vector3D predLOS, double range, ref Vector3D rawAccel)
        {
            var blendFwd = Vector3D.Normalize(forwardNorm * 0.8 + predLOS * 0.2);
            var maxAng   = MathHelper.Lerp(90.0, 30.0, MathHelper.Clamp(range / 300.0, 0.0, 1.0));

            // if we’re in hyper-aggressive cancel mode, switch to pure cancel
            if (rawAccel == /* flagged hyper-aggressive accel */) 
                return /* cancelAccel logic */;

            return LimitAngleFromForward(accel, blendFwd, maxForwardAccelCapability, maxAng);
        }

        private Vector3D ApplyGLimit(Vector3D accel)
        {
            var gLoad = accel.Length() / 9.81;
            if (gLoad > maxG)
                accel *= (maxG / gLoad);
            return accel;
        }

        #endregion


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

