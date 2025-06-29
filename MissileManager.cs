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
        public class MissileManager
        {
            public List<Missile> launchedMissiles = new List<Missile>();

            private Vector3D planetCenterPos;
            private double planetGravity;
            private double worldMaxSpeed;
            private double planetSeaLevelRadius;
            private double proximityDetonationDistance;
            private double proximityArmingDistance;

            public MissileManager(double worldMaxSpeed, double proximityArmingDistance, double proximityDetonationDistance)
            {
                this.worldMaxSpeed = worldMaxSpeed;
                this.proximityArmingDistance = proximityArmingDistance;
                this.proximityDetonationDistance = proximityDetonationDistance;
            }

            public IEnumerator<bool> LaunchMissile(
                IMyTerminalBlock referenceBlock,
                List<IMyFlightMovementBlock> movementBlocks,
                List<IMyOffensiveCombatBlock> combatBlocks,
                List<IMyShipMergeBlock> mergeBlocks,
                List<IMyGyro> gyros,
                List<IMyThrust> thrusters,
                List<IMyWarhead> warheads,
                List<IMyGasTank> gasTanks,
                List<IMyBatteryBlock> batteries,
                List<IMyShipConnector> connectors,
                DLBus.DLBusDetectedEntity target = null)
            {
                foreach (var mergeBlock in mergeBlocks)
                {
                    if (mergeBlock.IsConnected && mergeBlock.Enabled)
                    {
                        mergeBlock.Enabled = false;
                        break;
                    }
                }

                yield return true;

                Missile missile = new Missile(worldMaxSpeed, proximityDetonationDistance, proximityArmingDistance);

                for (int i = gyros.Count - 1; i >= 0; i--)
                {
                    if (!gyros[i].IsSameConstructAs(referenceBlock))
                    {
                        missile.gyros.Add(gyros[i]);
                        gyros.RemoveAt(i);
                    }
                }

                for (int i = connectors.Count - 1; i >= 0; i--)
                {
                    if (!connectors[i].IsSameConstructAs(referenceBlock))
                    {
                        missile.connectors.Add(connectors[i]);
                        connectors.RemoveAt(i);
                    }
                }

                for (int i = movementBlocks.Count - 1; i >= 0; i--)
                {
                    if (!movementBlocks[i].IsSameConstructAs(referenceBlock))
                    {
                        missile.flightMovementBlock = movementBlocks[i];
                        movementBlocks.RemoveAt(i);
                    }
                }

                for (int i = combatBlocks.Count - 1; i >= 0; i--)
                {
                    if (!combatBlocks[i].IsSameConstructAs(referenceBlock))
                    {
                        missile.offensiveCombatBlock = combatBlocks[i];
                        combatBlocks.RemoveAt(i);
                    }
                }

                for (int i = thrusters.Count - 1; i >= 0; i--)
                {
                    if (!thrusters[i].IsSameConstructAs(referenceBlock))
                    {
                        missile.thrusters.Add(thrusters[i]);
                        thrusters.RemoveAt(i);
                    }
                }

                for (int i = warheads.Count - 1; i >= 0; i--)
                {
                    if (!warheads[i].IsSameConstructAs(referenceBlock))
                    {
                        missile.warheads.Add(warheads[i]);
                        warheads.RemoveAt(i);
                    }
                }

                for (int i = gasTanks.Count - 1; i >= 0; i--)
                {
                    if (!gasTanks[i].IsSameConstructAs(referenceBlock))
                    {
                        missile.gasTanks.Add(gasTanks[i]);
                        gasTanks.RemoveAt(i);
                    }
                }

                for (int i = batteries.Count - 1; i >= 0; i--)
                {
                    if (!batteries[i].IsSameConstructAs(referenceBlock))
                    {
                        missile.batteries.Add(batteries[i]);
                        batteries.RemoveAt(i);
                    }
                }

                yield return true;
                missile.Initialize();
                

                if (planetGravity > 0)
                {
                    missile.UpdatePlanetValues(planetCenterPos, planetGravity, planetSeaLevelRadius);
                }

                if (target != null)
                    missile.UpdateTargetedEntity(target);

                if (missile.batteries.Count != 0 &&
                    missile.thrusters.Count != 0 &&
                    missile.gyros.Count != 0)
                {
                    launchedMissiles.Add(missile);    
                }
            }

            public void ManageMissiles(long currentPbTime)
            {
                for (int i = launchedMissiles.Count - 1; i >= 0; i--) {
                    if (launchedMissiles[i].Health != Missile.MissileHealth.Dead) {
                        try
                        {
                            launchedMissiles[i].Flight(currentPbTime);
                        }
                        catch (Exception)
                        {
                            launchedMissiles.RemoveAt(i);
                        }
                    }
                    else
                        launchedMissiles.RemoveAt(i);
                }
            }

            public void UpdateDetectionsDeferred(List<DLBus.DLBusDetectedEntity> updatedEntities)
            {
                foreach (var missile in launchedMissiles)
                {
                    if (missile.Health == Missile.MissileHealth.Dead)
                        continue;

                    foreach (var updatedEntity in updatedEntities)
                    {
                        if (missile.ExternalTarget == null)
                            continue;
                        
                        if (missile.ExternalTarget.EntityId == updatedEntity.EntityId)
                        {
                            missile.UpdateTargetedEntity(updatedEntity);
                        }
                    }
                }
            }

            public void UpdatePlanetValues(Vector3D planetCenter, double gravity, double sealevelRadius)
            {
                planetCenterPos = planetCenter;
                planetGravity = gravity;
                planetSeaLevelRadius = sealevelRadius;
            }
        }
    }
}

