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
            private double navP = 3.0f;
            private double maxAccelCapability = 83;
            private float interceptRadius = 25.0f; // avoid last-moment turns
            private float headingBiasWeight = 0.1f; // blend weight to bias toward target direction
            private double dampingFactor = 0.2f; // 0 = no damping, 1 = full damping

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
            private Vector3D previousAccel = Vector3D.Zero;

            private PIDController yawPID = new PIDController(5.0, 0.0, 1);
            private PIDController pitchPID = new PIDController(5.0, 0.0, 1);

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

                Vector3D accelCommand = ComputeGuidanceAccel(targetPos, targetVel);

                if (Vector3D.IsZero(accelCommand))
                {
                    // Fallback to current velocity direction if guidance fails
                    Vector3D fallbackDirection = Vector3D.Normalize(Velocity);
                    ThrustUtils.SetThrustBasedDot(thrusters, fallbackDirection);
                    AimInDirection(fallbackDirection, currentPbTime);
                    return;
                }

                Vector3D currentVelocityDir = Vector3D.Normalize(Velocity);
                Vector3D currentTargetDir = Vector3D.Normalize(targetPos - Position);
                Vector3D requiredAccelDir = Vector3D.Normalize(accelCommand + currentTargetDir * Math.Min(1, maxAccelCapability - accelCommand.Length()));

                // Calculate angle needed to achieve desired lateral acceleration
                double angleBetween = Math.Acos(MathHelper.Clamp(Vector3D.Dot(currentVelocityDir, requiredAccelDir), -1.0, 1.0));
                double maxTurnAngle = Math.Asin(Math.Min(accelCommand.Length() / maxAccelCapability, 1.0));

                // If more turn than required, limit it to necessary
                double limitedAngle = Math.Min(angleBetween, maxTurnAngle);
                Vector3D limitedDirection;
                if (angleBetween < 1e-6) // Avoid division by zero
                {
                    limitedDirection = currentVelocityDir;
                }
                else
                {
                    limitedDirection = Vector3D.Normalize(Vector3D.Lerp(currentVelocityDir, requiredAccelDir, limitedAngle / angleBetween));
                }

                ThrustUtils.SetThrustBasedDot(thrusters, limitedDirection); // Changed from requiredAccelDir
                AimInDirection(limitedDirection, currentPbTime); // Changed from requiredAccelDir
            }

            private Vector3D ComputeGuidanceAccel(Vector3D targetmissilePos, Vector3D targetVelocity)
            {
                Vector3D rangeVec = targetmissilePos - Position;
                Vector3D toTargetDir = Vector3D.Normalize(rangeVec);
                Vector3D relativeVelocity = targetVelocity - Velocity;

                Vector3D VelocityNorm = Velocity.LengthSquared() > 1e-6f ? Vector3D.Normalize(Velocity) : toTargetDir;

                double dotVelTarget = Vector3D.Dot(VelocityNorm, toTargetDir);
                double angleBetween = (double)Math.Acos(MathHelper.Clamp(dotVelTarget, -1f, 1f));
                double range = rangeVec.Length();

                if (range < interceptRadius)
                {
                    double timeToIntercept = range / Math.Max(1f, Velocity.Length());
                    Vector3D predictedTargetPos = targetmissilePos + relativeVelocity * (float)timeToIntercept;
                    Vector3D predictedDir = Vector3D.Normalize(predictedTargetPos - targetmissilePos);

                    Vector3D biasedDir = Vector3D.Normalize(Vector3D.Lerp(toTargetDir, predictedDir, headingBiasWeight));

                    return biasedDir  * maxAccelCapability;
                }
                else
                {
                    double rangeSq = rangeVec.LengthSquared();
                    double closingSpeed = -Vector3D.Dot(relativeVelocity, Vector3D.Normalize(toTargetDir));

                    Vector3D omegaL = Vector3D.Cross(toTargetDir, relativeVelocity) / (float)rangeSq;
                    Vector3D desiredAccel = navP * (float)closingSpeed * Vector3D.Cross(omegaL, Vector3D.Normalize(toTargetDir));

                    Vector3D lateralDesiredAccel = desiredAccel - Vector3D.Dot(desiredAccel, toTargetDir) * toTargetDir;

                    Vector3D biasedDir = Vector3D.Normalize(toTargetDir);
                    if (lateralDesiredAccel.LengthSquared() > 0.01f)
                    {
                        Vector3D desiredDirection = Vector3D.Normalize(lateralDesiredAccel);

                        biasedDir = Vector3D.Normalize(Vector3D.Lerp(desiredDirection, toTargetDir, headingBiasWeight));
                    }

                    Vector3D rawAccel = biasedDir * maxAccelCapability;

                    Vector3D dampedAccel = Vector3D.Lerp(previousAccel, rawAccel, 1.0f - dampingFactor);
                    previousAccel = dampedAccel;

                    return dampedAccel;
                }
            }

            private Vector3D CalculateLateralAccel(Vector3D rangeVec, Vector3D closingVelocity)
            {
                double rangeSquared = rangeVec.LengthSquared();
                if (rangeSquared < 1e-6f)
                {
                    return Vector3D.Zero;
                }

                double closingSpeed = -Vector3D.Dot(closingVelocity, Vector3D.Normalize(rangeVec));
                Vector3D omegaL = Vector3D.Cross(rangeVec, closingVelocity) / rangeSquared;
                Vector3D accelCommand = navP * closingSpeed * Vector3D.Cross(omegaL, Vector3D.Normalize(rangeVec));

                return accelCommand;
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

                    double currentTime = currentPbTime * (1.0 / 60.0); // or TimeSinceStart

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

                    double currentTime = currentPbTime * (1.0 / 60.0);

                    double yawOutput = yawPID.Compute(yaw, currentTime);
                    double pitchOutput = pitchPID.Compute(pitch, currentTime);

                    GyroUtils.ApplyGyroOverride(gyros, thrustDirectionReference.WorldMatrix, pitchOutput, yawOutput, 0);
                }
            }
        }
    }
}

