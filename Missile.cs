using BulletXNA;
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
using System.Numerics;
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
            private const double maxForwardAccelCapability = 100;
            private const double maxTurnRateRadPerSec = 6;
            private const double referenceTurnRate = 6.2;

            private const double maxPredictTime = 3;
            private const double maxAccelPredictTime = 2;
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
            private PIDController yawPID = new PIDController(15, 0.0, 0.1);
            private PIDController pitchPID = new PIDController(15, 0.0, 0.1);

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

            Vector3D targetPos;
            Vector3D targetVel;
            Vector3D targetAcc;

            double closestDist = 5000;
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

                targetPos = radarTrackingModule.TargetPosition;
                targetAcc = radarTrackingModule.TargetVelocity - targetAcc;
                targetVel = radarTrackingModule.TargetVelocity;


                UpdateRadarRefreshRate(Vector3D.Distance(targetPos, Position));

                Vector3D gravitydirection = planetCenterPos - Position;
                double altitude = gravitydirection.Normalize();


                Vector3D accelCommand = ComputeGuidanceAccel(targetPos, targetVel, targetAcc, Position, Velocity, DeltaTime) - gravitydirection * planetGravity;
                double accelMag = Math.Min(accelCommand.Normalize(), maxForwardAccelCapability - 10);
                double leftoverAccel = Math.Max(10, maxForwardAccelCapability - accelMag);

                Vector3D targetDir = targetPos - Position;
                double targetDist = targetDir.Normalize();
                if (targetDist < closestDist)
                    closestDist = targetDist;

                accelCommand = (accelCommand * accelMag) + (targetDir * leftoverAccel);
                Vector3D lookCommand = accelCommand;

                double dot = Vector3D.Dot(Forward, Vector3D.Normalize(accelCommand));
                dot = MathHelper.Clamp(dot, -1.0, 1.0); // Clamp is critical for acos domain
                double angleDegrees = MathHelper.ToDegrees(Math.Acos(dot));

                Program.globalScreamValue = $"commanded:{accelMag:N3}\ndeg:{angleDegrees}\nleftover: {leftoverAccel:N3}\ntotal: {accelCommand.Length():N3}\ndist: {targetDist:N2}\nclosest: {closestDist:N2}";

                ThrustUtils.SetThrustBasedDot(thrusters, Vector3D.Normalize(accelCommand)); // Changed from requiredAccelDir
                AimInDirection(Vector3D.Normalize(lookCommand), currentPbTime); // Changed from requiredAccelDir
            }

            Vector3D previousAccel;

            private Vector3D ComputeGuidanceAccel(
                Vector3D targetPos,
                Vector3D targetVel,
                Vector3D targetAccel, // estimated or calculated externally
                Vector3D missilePos,
                Vector3D missileVel,
                double deltaTime
            )
            {
                // --- Estimate intercept time (simplified) ---
                Vector3D relPos = targetPos - missilePos;
                Vector3D relVel = targetVel - missileVel;
                double distance = relPos.Length();
                double relSpeed = relVel.Length();

                // Avoid divide-by-zero and negative relative speeds
                double timeToIntercept = (relSpeed > 1e-2) ? distance / relSpeed : 0.0;
                double predictionTime = Math.Min(timeToIntercept, maxPredictTime);

                // --- Predict future target state ---
                // s = ut + 0.5at²
                Vector3D predictedTargetPos = targetPos + targetVel * predictionTime + 0.5 * targetAccel * predictionTime * predictionTime;
                Vector3D predictedTargetVel = targetVel + targetAccel * Math.Min(maxAccelPredictTime, predictionTime);
                if (predictedTargetVel.LengthSquared() > 100 * 100)
                    predictedTargetVel = Vector3D.Normalize(predictedTargetVel) * 100;

                // --- Recompute guidance using predicted target state ---
                Vector3D newRelPos = predictedTargetPos - missilePos;
                double distanceSq = newRelPos.LengthSquared();
                double newDistance = Math.Sqrt(distanceSq);

                if (distanceSq < 1e-4) return Vector3D.Zero; // avoid instability

                Vector3D newRelVel = predictedTargetVel - missileVel;
                Vector3D los = newRelPos / newDistance;
                double closingVel = -Vector3D.Dot(newRelVel, los);

                Vector3D losRate = (distanceSq > 1e-4)
                    ? Vector3D.Cross(newRelPos, newRelVel) / distanceSq
                    : Vector3D.Zero;

                // Adaptive navigation constant
                double navigationConstant = MathHelper.Lerp(3.0, 6.0, MathHelper.Clamp(newDistance / 2000.0, 0, 1));

                Vector3D pnAccel = navigationConstant * closingVel * Vector3D.Cross(losRate, los);

                Vector3D targetAccelPerp = targetAccel - los * Vector3D.Dot(targetAccel, los);

                Vector3D accelCmd = pnAccel + targetAccelPerp;

                // Clamp to max acceleration
                double accelMagSq = accelCmd.LengthSquared();
                if (accelMagSq > maxForwardAccelCapability * maxForwardAccelCapability)
                {
                    accelCmd = Vector3D.Normalize(accelCmd) * maxForwardAccelCapability;
                }

                // Smooth output using exponential decay
                double smoothingFactor = Math.Exp(-deltaTime / 0.2);
                accelCmd = Vector3D.Lerp(accelCmd, previousAccel, smoothingFactor);

                previousAccel = accelCmd;

                return accelCmd;
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



            Vector3D previousDirection;
            private void AimInDirection(Vector3D aimDirection, long currentPbTime)
            {
                double overshootCompensation = 0.9;
                

                if (thrustDirectionReference == null || !thrustDirectionReference.IsFunctional)
                    return;
                var refMatrix = thrustDirectionReference.WorldMatrix;

                Vector3D diffDirection = Vector3D.Normalize(refMatrix.Forward - previousDirection);
                Vector3D diffTarget = Vector3D.Normalize(aimDirection - refMatrix.Forward);

                Vector3D difference = Vector3D.Normalize(diffDirection - diffTarget);
                Vector3D compensatedAimDir = Vector3D.Reflect(difference, aimDirection) * overshootCompensation;

                double dot = Vector3D.Dot(Forward, diffDirection);
                if (dot < 0.95)
                    dot = 0;

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
                    Vector3D.Normalize(aimDirection + difference * dot * overshootCompensation),
                    out yawError,
                    out pitchError
                );

                // ——————————————————————————————————————————————
                // 3) PID loops
                // ——————————————————————————————————————————————
                double rawYaw = yawPID.Compute(yawError, t);
                double rawPitch = pitchPID.Compute(pitchError, t);

                const double maxGyro = Math.PI*2;
                double yawOut = MathHelper.Clamp(rawYaw, -maxGyro, maxGyro);
                double pitchOut = MathHelper.Clamp(rawPitch, -maxGyro, maxGyro);

                GyroUtils.ApplyGyroOverride(gyros, refMatrix, pitchOut, yawOut, 0);

                previousDirection = refMatrix.Forward;
            }
            
            public static void DirectionToPitchYaw(
                Vector3D forward,   // ship’s forward
                Vector3D right,     // ship’s right (not left!)
                Vector3D up,        // ship’s up
                Vector3D targetDir, // normalized aim direction
                out double yaw,
                out double pitch
            )
            {
                // 1) split target into “up” vs “horizontal” parts
                var upComp = VectorUtils.Project(targetDir, up);
                var horizComp = targetDir - upComp;

                // 2) angles between forward/horizontal and full/ horizontal
                yaw = VectorUtils.GetAngle(forward, horizComp);
                pitch = VectorUtils.GetAngle(targetDir, horizComp);

                // 3) sign‐bit: if target is to your right, positive yaw; to your left, negative
                yaw *= -Math.Sign(right.Dot(targetDir));
                //    if target is above your plane it's positive pitch, else negative
                pitch *= -Math.Sign(up.Dot(targetDir));

                // 4) exactly behind you?
                if (yaw == 0 && pitch == 0 && targetDir.Dot(forward) < 0)
                    yaw = Math.PI;
            }

        }
    }
}

