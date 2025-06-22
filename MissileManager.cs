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

                Missile missile = new Missile();

                for (int i = gyros.Count - 1; i >= 0; i--)
                {
                    if (!gyros[i].IsSameConstructAs(referenceBlock))
                    {
                        missile.gyros.Add(gyros[i]);
                        gyros.RemoveAt(i);
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

                if (target != null)
                    missile.UpdateTargetedEntity(target);

                launchedMissiles.Add(missile);
            }

            public void ManageMissiles(long currentPbTime)
            {
                foreach (var missile in launchedMissiles)
                {
                    if (missile.Health != Missile.MissileHealth.Dead)
                        missile.Flight(currentPbTime);
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
                        if (missile.ExternalTarget.EntityId == updatedEntity.EntityId)
                        {
                            missile.UpdateTargetedEntity(updatedEntity);
                        }
                    }
                }
            }

            public void UpdatePlanetValuses(Vector3D planetCenter, double gravity)
            {
                foreach (var missile in launchedMissiles)
                {
                    missile.UpdatePlanetValues(planetCenter, gravity);
                }
            }
        }
    }
}

