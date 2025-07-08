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

            private double maxForwardAccelCapability = 100;
            private double minimumP = 3;
            private double maximumP = 5;
            private const double minimumClosingVel = -10;

            private const double DeltaTime = 1.0 / 60.0;
            private const double predictionTimeCap = 30;

            private const double CruisingHeight = 1.15;
            private const long targetUpdatedTimeoutSeeker = 60 * 60;

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

            public enum MissileLockType
            {
                External,
                Internal,
                None,
            }

            public List<IMyThrust> thrusters = new List<IMyThrust>();
            public List<IMyGyro> gyros = new List<IMyGyro>();
            public List<IMyWarhead> warheads = new List<IMyWarhead>();
            public List<IMyGasTank> gasTanks = new List<IMyGasTank>();
            public List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();
            public List<IMyShipConnector> connectors = new List<IMyShipConnector>();

            public IMyFlightMovementBlock flightMovementBlock;
            public IMyOffensiveCombatBlock offensiveCombatBlock;

            public MissileFlightState FlightState { get; private set; }
            public MissileHealth Health { get; private set; }
            public MissileLockType LockType { get; private set; }
            public double FuelRemainingFraction { get; private set; }
            public long lifeTimeCounter { get; private set; }
            public double OriginalLaunchDistance;
            public double CurrentTargetDistance;

            public Vector3D Position { get; private set; }
            public Vector3D Velocity { get; private set; }
            public Vector3D Forward => GetForward();

            public DLBus.DLBusDetectedEntity ExternalTarget { get; private set; }

            private Vector3D launchForward;
            private Vector3D planetCenterPos;
            private double planetGravity;
            private double planetseaLevelRadius;
            private RadarTrackingModule radarTrackingModule;
            private Vector3D majorityThrustDirectionLocal;
            private IMyTerminalBlock thrustDirectionReference;

            private PIDController yawPID = new PIDController(6, 0.0, 0.0);
            private PIDController pitchPID = new PIDController(6, 0.0, 0.0);

            private double worldMaxSpeed;
            private double proximityDetonationDistance = 5;
            private double proximityArmingDistance = 25;

            private DebugAPI debug;

            public Missile(double worldMaxSpeed, double proximityDetonationDistance, double proximityArmingDistance, DebugAPI debug)
            {
                this.debug = debug;
                this.worldMaxSpeed = worldMaxSpeed;
                this.FlightState = MissileFlightState.Unknown;
                this.Health = MissileHealth.Unknown;
                this.proximityDetonationDistance = proximityDetonationDistance;
                this.proximityArmingDistance = proximityArmingDistance;
            }

            public bool Initialize(double missileMass = 0)
            {
                if (flightMovementBlock != null && offensiveCombatBlock != null)
                {
                    radarTrackingModule = new RadarTrackingModule(flightMovementBlock, offensiveCombatBlock);
                    radarTrackingModule.ForcedRefreshRate = 10;
                }

                lifeTimeCounter = 0;

                foreach (var connector in connectors)
                {
                    connector.Enabled = true;
                    connector.Disconnect();
                }

                foreach (var gyro in gyros)
                {
                    gyro.Enabled = true;
                }

                foreach (var thruster in thrusters)
                {
                    thruster.Enabled = true;
                }

                foreach (var gastank in gasTanks)
                {
                    gastank.Enabled = true;
                    gastank.Stockpile = false;
                }

                foreach (var battery in batteries)
                {
                    battery.Enabled = true;
                    battery.ChargeMode = ChargeMode.Auto;
                }

                UpdateMissileHealth();
                majorityThrustDirectionLocal = GetMajorityThrustDirectionLocal(thrusters);

                if (missileMass != 0)
                {
                    double thrustAmount = ThrustUtils.GetForwardThrust(thrusters, Forward);
                    maxForwardAccelCapability = thrustAmount / missileMass;
                }

                if (Health == MissileHealth.Healthy || Health == MissileHealth.Degraded)
                {
                    FlightState = MissileFlightState.Launching;
                    return true;
                }

                return false;
            }

            public void UpdatePlanetValues(Vector3D center, double gravity, double sealevelRadius)
            {
                planetCenterPos = center;
                planetGravity = gravity;
                planetseaLevelRadius = sealevelRadius;
            }

            public void Flight(long currentPbTime)
            {
                if (radarTrackingModule != null)
                    radarTrackingModule.UpdateTracking(currentPbTime);

                UpdateVelocityPosition(currentPbTime);

                if (lifeTimeCounter % 60 == 0)
                {
                    UpdateMissileHealth();
                    UpdateFuelCounter();
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

            private void UpdateFuelCounter()
            {
                double total = 0;

                if (gasTanks.Count > 0)
                {
                    foreach (var tank in gasTanks)
                    {
                        if (tank.IsFunctional)
                        {
                            total += tank.FilledRatio;
                        }
                    }

                    FuelRemainingFraction = total / (double)gasTanks.Count;
                    return;
                }

                foreach (var battery in batteries)
                {
                    if (battery.IsFunctional)
                    {
                        total += battery.CurrentStoredPower / battery.MaxStoredPower;
                    }
                }

                FuelRemainingFraction = total / (double)batteries.Count;
            }

            private void FlightLaunching(long currentPbTime)
            {
                if (lifeTimeCounter < 60 * 1)
                {
                    if (launchForward == Vector3D.Zero)
                        launchForward = Forward;

                    AimInDirection(launchForward, currentPbTime);
                    ThrustUtils.SetThrustBasedDot(thrusters, launchForward);
                }
                else if (lifeTimeCounter < 60 * 5 && planetGravity != 0)
                {
                    long localCounter = lifeTimeCounter - 60 * 1;

                    double ratio = (double)localCounter / (60.0 * 4);

                    Vector3D planetUpDirection = Vector3D.Normalize(Position - planetCenterPos);

                    Vector3D aimVector = Vector3D.Normalize(launchForward + (ratio * planetUpDirection));

                    AimInDirection(aimVector, currentPbTime);
                }
                else
                {
                    FlightState = MissileFlightState.Cruising;
                }
            }

            Vector3D previousAccel;
            private void FlightCruising(long currentPbTime)
            {
                if (ExternalTarget != null)
                {
                    LockType = MissileLockType.External;

                    Vector3D predictedExternalPosition = ExternalTarget.LastKnownLocation + (ExternalTarget.LastKnownVelocity);
                    Vector3D distanceFromTarget = predictedExternalPosition - Position;
                    double distanceSquared = distanceFromTarget.LengthSquared();

                    UpdateRadarRefreshRate(distanceSquared);
                    CurrentTargetDistance = distanceFromTarget.Length();

                    if (OriginalLaunchDistance == 0)
                        OriginalLaunchDistance = CurrentTargetDistance;

                    Vector3D closingVelocity = Velocity - ExternalTarget.LastKnownVelocity * 60;
                    Vector3D roughTargetLocationPrediction = predictedExternalPosition * (closingVelocity.LengthSquared() / distanceSquared);

                    if (planetGravity > 1e-3 && distanceSquared > 2500 * 2500)
                    {
                        // 1) Radial basis
                        Vector3D toCenter = planetCenterPos - Position;
                        double currentDist = toCenter.Length();
                        Vector3D radialDir = Vector3D.Normalize(toCenter);
                        double desiredDist = planetseaLevelRadius * CruisingHeight;

                        // 2) Great-circle tangent direction toward target
                        Vector3D toTarget = predictedExternalPosition - Position;
                        double targetAltitude = toTarget.Normalize();
                        Vector3D greatCircleDir = Vector3D.Normalize(Vector3D.Cross(Vector3D.Cross(radialDir, toTarget), radialDir));
                        Vector3D tangent = Vector3D.Normalize(greatCircleDir);

                        // 3) Velocity‑error P‑term
                        double kP = 0.75;
                        Vector3D v_err = tangent * worldMaxSpeed - Velocity;
                        Vector3D a_tangent = v_err * kP;

                        double radialDot = Vector3D.Dot(radialDir, toTarget);
                        if (radialDot > 0.8)
                        {
                            FlightState = MissileFlightState.Terminal;
                        }

                        // 4) Gravity/centripetal compensation
                        //    If planetGravity is your local g‑pull (positive number), it points _inward_ so we need to negate it to push outward:
                        Vector3D a_gravity = -radialDir * (planetGravity + (desiredDist - currentDist));

                        // 5) Total accel, smooth, normalize for thrust direction
                        Vector3D accelUnsmoothed = a_tangent + a_gravity;
                        Vector3D accelCommand = Vector3D.Lerp(previousAccel, accelUnsmoothed, 0.3);
                        previousAccel = accelCommand;

                        Vector3D thrustDir = Vector3D.Normalize(accelCommand);

                        // 6) Thrust on/off based on tangential speed
                        double speedAlongTangent = Vector3D.Dot(Velocity, tangent);
                        if (speedAlongTangent > worldMaxSpeed * 0.97)
                            ThrustUtils.SetThrustPercentage(thrusters, 0f);
                        else
                            ThrustUtils.SetThrustBasedDot(thrusters, thrustDir);

                        // 7) Aim where you thrust
                        AimInDirection(thrustDir, currentPbTime);
                    }
                    else
                    {
                        Vector3D directionToTarget = Vector3D.Normalize(distanceFromTarget);
                        AimInDirection(directionToTarget, currentPbTime);

                        if (Velocity.LengthSquared() > 95 * 95 && Velocity.Dot(directionToTarget) > 0.95)
                        {
                            ThrustUtils.SetThrustPercentage(thrusters, 0f);
                        }
                        else
                        {
                            ThrustUtils.SetThrustBasedDot(thrusters, directionToTarget);
                        }
                    }

                    if (distanceSquared < 2500 * 2500 || (currentPbTime - ExternalTarget.DetectionReceivedTime) > targetUpdatedTimeoutSeeker)
                    {
                        FlightState = MissileFlightState.Terminal;
                    }
                }
                else
                {
                    LockType = MissileLockType.None;

                    if (planetGravity > 1e-3)
                    {
                        // 1) Radial basis
                        Vector3D toCenter = planetCenterPos - Position;
                        double currentDist = toCenter.Length();
                        Vector3D radialDir = Vector3D.Normalize(toCenter);
                        double desiredDist = planetseaLevelRadius * CruisingHeight;

                        // 2) True tangential dir from your current velocity
                        Vector3D v_tang_un = Velocity - radialDir * Vector3D.Dot(Velocity, radialDir);
                        Vector3D tangent = Vector3D.Normalize(v_tang_un);

                        // 3) Velocity‑error P‑term
                        double kP = 0.75;
                        Vector3D v_err = tangent * worldMaxSpeed - Velocity;
                        Vector3D a_tangent = v_err * kP;

                        // 4) Gravity/centripetal compensation
                        //    If planetGravity is your local g‑pull (positive number), it points _inward_ so we need to negate it to push outward:
                        Vector3D a_gravity = -radialDir * (planetGravity + (desiredDist - currentDist));

                        // 5) Total accel, smooth, normalize for thrust direction
                        Vector3D accelUnsmoothed = a_tangent + a_gravity;
                        Vector3D accelCommand = Vector3D.Lerp(previousAccel, accelUnsmoothed, 0.3);
                        previousAccel = accelCommand;

                        Vector3D thrustDir = Vector3D.Normalize(accelCommand);

                        // 6) Thrust on/off based on tangential speed
                        double speedAlongTangent = Vector3D.Dot(Velocity, tangent);
                        if (speedAlongTangent > worldMaxSpeed * 0.97)
                            ThrustUtils.SetThrustPercentage(thrusters, 0f);
                        else
                            ThrustUtils.SetThrustBasedDot(thrusters, thrustDir);

                        // 7) Aim where you thrust
                        AimInDirection(thrustDir, currentPbTime);

                    }
                    else
                    {
                        if (Velocity.LengthSquared() > 95 * 95 && Velocity.Dot(launchForward) > 0.99)
                        {
                            ThrustUtils.SetThrustPercentage(thrusters, 0f);
                        }
                        else
                        {
                            ThrustUtils.SetThrustBasedDot(thrusters, launchForward);
                        }

                        AimInDirection(launchForward, currentPbTime);
                    }

                    if (radarTrackingModule.IsTracking)
                        FlightState = MissileFlightState.Terminal;
                }
            }


            Vector3D targetPos;
            Vector3D targetVel;

            double closestDist = 5000;
            private void FlightTerminal(long currentPbTime)
            {
                LockType = MissileLockType.Internal;

                if (radarTrackingModule == null || !radarTrackingModule.IsTracking)
                {
                    LockType = MissileLockType.None;
                    // No target, fly straight
                    Vector3D fallbackDirection = Vector3D.Normalize(Velocity);

                    if (ExternalTarget != null && (currentPbTime - ExternalTarget.DetectionReceivedTime) < targetUpdatedTimeoutSeeker)
                    {
                        LockType = MissileLockType.External;
                        fallbackDirection = Vector3D.Normalize(ExternalTarget.LastKnownLocation - Position);
                    }
                    ThrustUtils.SetThrustBasedDot(thrusters, fallbackDirection);
                    AimInDirection(fallbackDirection, currentPbTime);
                    return;
                }

                targetPos = radarTrackingModule.GetTargetPosition(out targetVel);
                targetVel *= 60;

                if (Program.DEBUG_VERSION)
                {
                    debug.DrawPoint(targetPos, Color.Red, seconds: 1, onTop: true);
                    debug.DrawLine(targetPos, targetPos + targetVel, Color.Orange, seconds: 1, onTop: true);
                }

                UpdateRadarRefreshRate(Vector3D.DistanceSquared(targetPos, Position));

                Vector3D gravitydirection = planetCenterPos - Position;
                double altitude = gravitydirection.Normalize();

                Vector3D accelCommand = ComputeGuidanceAccel(targetPos, targetVel, Position, Velocity, DeltaTime) - gravitydirection * planetGravity;
                double accelMag = Math.Min(accelCommand.Normalize(), maxForwardAccelCapability - 10);
                double leftoverAccel = Math.Max(10, maxForwardAccelCapability - accelMag);

                Vector3D targetDir = targetPos - Position;
                double targetDist = targetDir.Normalize();
                double closingVelocity = -Vector3D.Dot(targetVel - Velocity, targetDir);
                double timeToTarget = Math.Min(predictionTimeCap, Math.Max(0.1, targetDist / closingVelocity));

                Vector3D predictedPos = targetPos + targetVel * timeToTarget;
                Vector3D predictedDir = Vector3D.Normalize(predictedPos - Position);

                CurrentTargetDistance = targetDist;
                if (targetDist < closestDist)
                    closestDist = targetDist;

                if (targetDist < proximityDetonationDistance)
                {
                    Detonate();
                }
                else if (targetDist < proximityArmingDistance)
                {
                    Arm();
                }


                accelCommand = (accelCommand * accelMag) + (predictedDir * leftoverAccel);
                Vector3D lookCommand = accelCommand;
                Vector3D accelDir = Vector3D.Normalize(accelCommand);

                double dot = Vector3D.Dot(Forward, accelDir);
                dot = MathHelper.Clamp(dot, -1.0, 1.0); // Clamp is critical for acos domain
                double angleDegrees = MathHelper.ToDegrees(Math.Acos(dot));

                if (Vector3D.Dot(predictedDir, Vector3D.Normalize(Velocity)) < 0.2)
                {
                    accelDir = predictedDir;
                }

                ThrustUtils.SetThrustBasedDot(thrusters, accelDir);

                AimInDirection(Vector3D.Normalize(lookCommand), currentPbTime); // Changed from requiredAccelDir

                if (debug != null)
                {
                    debug.PrintChat($"dist: {targetDist:N2} relVel: {closingVelocity:N2} TTT: {timeToTarget:N2}");
                }
            }

            private void Arm()
            {
                foreach (var warhead in warheads)
                {
                    if (warhead.Closed)
                        continue;

                    warhead.IsArmed = true;
                }
            }

            private void Detonate()
            {
                foreach (var warhead in warheads)
                {
                    if (warhead.Closed)
                        continue;

                    warhead.IsArmed = true;
                    warhead.Detonate();

                    Health = MissileHealth.Dead;
                }
            }


            private Vector3D ComputeGuidanceAccel(
                Vector3D targetPos,
                Vector3D targetVel,
                Vector3D missilePos,
                Vector3D missileVel,
                double deltaTime
            )
            {
                // --- Estimate intercept time (simplified) ---
                Vector3D relPos = targetPos - missilePos;
                Vector3D relVel = targetVel - missileVel;
                double distance = relPos.Length();

                if (distance < 1e-4) return Vector3D.Zero; // avoid instability

                Vector3D newRelVel = targetVel - missileVel;
                Vector3D los = relPos / distance;
                double closingVel = -Vector3D.Dot(newRelVel, los);

                Vector3D losRate = (distance > 1e-4)
                    ? Vector3D.Cross(relPos, newRelVel) / (distance * distance)
                    : Vector3D.Zero;

                // Adaptive navigation constant
                double navigationConstant = MathHelper.Lerp(minimumP, maximumP, MathHelper.Clamp(distance / 2000.0, 0, 1));

                Vector3D accelCmd = navigationConstant * closingVel * Vector3D.Cross(losRate, los);

                return accelCmd;
            }



            private void UpdateRadarRefreshRate(double distanceToTargetSquared)
            {
                if (radarTrackingModule == null)
                    return;

                if (distanceToTargetSquared > 3000 * 3000)
                {
                    radarTrackingModule.ForcedRefreshRate = 25;
                }
                else if (distanceToTargetSquared > 1500 * 1500)
                {
                    radarTrackingModule.ForcedRefreshRate = 10;
                }
                else if (distanceToTargetSquared > 1000 * 1000)
                {
                    radarTrackingModule.ForcedRefreshRate = 5;
                }
                else if (distanceToTargetSquared > 500 * 500)
                {
                    radarTrackingModule.ForcedRefreshRate = 2;
                }
                else
                {
                    radarTrackingModule.ForcedRefreshRate = 1;
                }
            }

            public void UpdateMissileHealth()
            {
                int thrusterWorkingCount = thrusters.Count(x => x.IsFunctional && !x.Closed);

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

                int gyroWorkingCount = gyros.Count(x => x.IsFunctional && !x.Closed);
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

                int warheadWorkingCount = warheads.Count(x => x.IsFunctional && !x.Closed);

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

                int gasTankWorkingCount = gasTanks.Count(x => x.IsFunctional && !x.Closed);

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

                int batteryWorkingCount = batteries.Count(x => x.IsFunctional && !x.Closed);

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

                Health = MissileHealth.Healthy;
            }

            private void UpdateVelocityPosition(long currentPbTime)
            {
                foreach (var gyro in gyros)
                {
                    if (gyro.IsFunctional)
                    {
                        Position = gyro.CubeGrid.GetPosition();
                        Velocity = gyro.CubeGrid.LinearVelocity;
                        break;
                    }
                }
            }

            private Vector3D GetForward()
            {
                if (thrustDirectionReference != null && !thrustDirectionReference.Closed)
                {
                    return VectorUtils.TransformDirLocalToWorld(thrustDirectionReference.WorldMatrix, majorityThrustDirectionLocal);
                }
                else if (flightMovementBlock != null && !flightMovementBlock.Closed)
                {
                    return flightMovementBlock.WorldMatrix.Forward;
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
                if (flightMovementBlock == null || !flightMovementBlock.IsFunctional)
                    return;

                var refMatrix = flightMovementBlock.WorldMatrix;
                double t = currentPbTime * DeltaTime;
                Vector3D forward = refMatrix.Forward;
                Vector3D up = refMatrix.Up;
                Vector3D left = refMatrix.Left;

                if (debug != null)
                    debug.DrawLine(refMatrix.Translation, refMatrix.Translation + aimDirection * 10, Color.LimeGreen, seconds: 0.1f);

                Vector3D PitchVector = Vector3D.Normalize(VectorUtils.ProjectOnPlane(left, aimDirection));
                Vector3D YawVector = Vector3D.Normalize(VectorUtils.ProjectOnPlane(up, aimDirection));

                double pitchAngle = VectorUtils.SignedAngle(PitchVector, forward, left);
                double yawAngle = VectorUtils.SignedAngle(YawVector, forward, up);

                double pitchInput = pitchPID.Compute(pitchAngle, t);
                double yawInput = yawPID.Compute(yawAngle, t);

                GyroUtils.ApplyGyroOverride(gyros, refMatrix, pitchInput, yawInput, 0);
            }
        }
    }
}

