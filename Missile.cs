using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
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
            private const double ExternalToRadarToleranceSquared = 2000 * 2000;
            private const double NavP = 10;
            private const double NavD = 1;
            private const double NavI = 0;

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
            private Vector3D oldAccelTarget;
            private Vector3D majorityThrustDirectionLocal;
            private IMyTerminalBlock thrustDirectionReference;

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
                    radarTrackingModule.RefreshRate = 10;
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

            public void UpdatePlanetValues(Vector3D center, double gravity) {
                planetCenterPos = center;
                planetGravity = gravity;
            }

            public void Flight(long currentPbTime)
            {
                if (radarTrackingModule != null)
                    radarTrackingModule.UpdateTracking(currentPbTime);

                UpdateMissileVelocityPosition(currentPbTime);

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
                    AimInDirection(Forward);
                    ThrustUtils.SetThrustBasedDot(thrusters, Forward);
                }
                else if (lifeTimeCounter < 60 * 5 && planetGravity != 0)
                {
                    long localCounter = lifeTimeCounter - 60 * 1;

                    double ratio = (double)localCounter / (60.0 * 4);

                    Vector3D planetDirection = Vector3D.Normalize(Position - planetCenterPos);

                    Vector3D aimVector = Vector3D.Normalize(Forward + (ratio * planetDirection));

                    AimInDirection(aimVector);
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
                }

                // TODO
                FlightState = MissileFlightState.Terminal;
            }

            private void FlightTerminal(long currentPbTime)
            {
                Vector3D distanceFromTarget = radarTrackingModule.TargetPosition - Position;
                UpdateRadarRefreshRate(distanceFromTarget.LengthSquared());

                Vector3D closingVelocity = Velocity - radarTrackingModule.TargetVelocity;

                Vector3D pnavDirection = CalculateLateralAccel(distanceFromTarget, closingVelocity);

                Vector3D aimDir = Vector3D.Normalize(pnavDirection + distanceFromTarget);

                AimInDirection(aimDir);
                ThrustUtils.SetThrustBasedDot(thrusters, aimDir);
            }

            private void UpdateRadarRefreshRate(double distanceToTargetSquared)
            {
                if (radarTrackingModule == null)
                    return;

                if (distanceToTargetSquared > 3000 * 3000)
                    {
                        radarTrackingModule.RefreshRate = 50;
                    }
                    else if (distanceToTargetSquared > 1500 * 1500)
                    {
                        radarTrackingModule.RefreshRate = 10;
                    }
                    else if (distanceToTargetSquared > 1000 * 1000)
                    {
                        radarTrackingModule.RefreshRate = 5;
                    }
                    else
                    {
                        radarTrackingModule.RefreshRate = 1;
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

            private void UpdateMissileVelocityPosition(long currentPbTime)
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

            private Vector3D CalculateLateralAccel(Vector3D rangeVec, Vector3D closingVelocity)
            {
                // Calculate rotation vec
                Vector3D RxV = Vector3D.Cross(rangeVec, closingVelocity);
                Vector3D RdR = rangeVec * rangeVec;
                Vector3D rotVec = RxV / RdR;

                // Pronav term
                Vector3D accelerationNormal = (closingVelocity).Cross(rotVec);

                Vector3D deltaError = accelerationNormal - oldAccelTarget;

                oldAccelTarget = accelerationNormal;
                return (NavP * accelerationNormal + NavD * deltaError);
            }

            private void AimInDirection(Vector3D aimdirection)
            {
                double yaw, pitch;

                if (flightMovementBlock != null && !flightMovementBlock.Closed)
                {
                    GyroUtils.DirectionToPitchYaw(
                        flightMovementBlock.WorldMatrix.Forward,
                        flightMovementBlock.WorldMatrix.Left, flightMovementBlock.WorldMatrix.Up,
                        aimdirection, out yaw, out pitch
                    );

                    GyroUtils.ApplyGyroOverride(gyros, flightMovementBlock.WorldMatrix, pitch * 1 , yaw * 1 , 0);
                }
                else if (thrustDirectionReference != null && thrustDirectionReference.IsFunctional)
                {
                    GyroUtils.DirectionToPitchYaw(thrustDirectionReference.WorldMatrix.Forward,
                        thrustDirectionReference.WorldMatrix.Left, thrustDirectionReference.WorldMatrix.Up,
                        aimdirection, out yaw, out pitch
                    );
                    
                    GyroUtils.ApplyGyroOverride(gyros, thrustDirectionReference.WorldMatrix, pitch * 1 , yaw * 1, 0);
                }
            }
        }
    }
}

