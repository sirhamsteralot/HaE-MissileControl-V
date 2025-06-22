using Sandbox.Game.EntityComponents;
using Sandbox.Game.Screens.Helpers;
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
        public partial class DLBus
        {
            private class DLBusLaserCommuncationClient
            {
                public struct CommsSatelite
                {
                    public long SateliteId;
                    public long LastUpdateTime;
                    public Vector3D Position;
                    public List<Vector3D> OpenAntennaPositions;
                    public List<long> ConnectedTo;
                }

                public List<IMyLaserAntenna> laserAntennas;

                public Dictionary<long, CommsSatelite> SatteliteLocations = new Dictionary<long, CommsSatelite>();

                public HashSet<long> ConnectedTo = new HashSet<long>();
                public long MyId;
                public Vector3D MyPosition;

                public const int minConnections = 2;
                public const int desiredConnections = 3;

                public DLBusLaserCommuncationClient(List<IMyLaserAntenna> laserAntennas, long myId, Vector3D myPosition)
                {
                    this.laserAntennas = laserAntennas;
                    MyId = myId;
                    MyPosition = myPosition;
                }

                public int ActiveConnectionCount()
                {
                    int count = 0;
                    foreach (var antenna in laserAntennas)
                    {
                        if ((antenna.Status | MyLaserAntennaStatus.Connected) == MyLaserAntennaStatus.Connected)
                            count++;
                    }

                    return count;
                }

                public void ReceiveAdvertisement(long IGCId, CommsSatelite satelite)
                {
                    if (IGCId != MyId)
                        SatteliteLocations[IGCId] = satelite;
                }

                public int ScoreConnection(long satId) {
                    int score = 0;

                    int xConnectedCount = SatteliteLocations[satId].ConnectedTo.Count;

                    if (xConnectedCount >= minConnections)
                    {
                        score -= (xConnectedCount * 10);

                        double distanceX = (SatteliteLocations[satId].Position - MyPosition).LengthSquared();
                        score -= (int)Math.Floor(distanceX / 10000.0);
                    }
                    else
                    {
                        score = 1000;
                    }

                    return score;
                }

                public void RefreshConnection(long currentTick)
                {
                    ConnectedTo.Clear();

                    List<long> SatteliteConnectionPriorityScore = SatteliteLocations.Keys
                        .OrderByDescending(id => ScoreConnection(id))
                        .ToList();

                    int satIndex = 0;

                    foreach (var laserAntenna in laserAntennas)
                    {
                        if (!laserAntenna.IsFunctional)
                            continue;

                        if (laserAntenna.Status == MyLaserAntennaStatus.Connected)
                            {
                                var orderedList = SatteliteLocations.Values
                                    .OrderBy(x => (x.Position - laserAntenna.TargetCoords).LengthSquared());

                            if (!orderedList.Any())
                                break;
                            
                            if (ConnectedTo.Add(orderedList.First().SateliteId))
                                    continue;
                            }

                        while (satIndex < SatteliteConnectionPriorityScore.Count)
                        {
                            long connectTo = SatteliteConnectionPriorityScore[satIndex];
                            CommsSatelite connectToSat = SatteliteLocations[connectTo];
                            satIndex++;

                            if (connectToSat.OpenAntennaPositions.Count == 0 || ConnectedTo.Contains(connectTo))
                                continue;

                            laserAntenna.SetTargetCoords(GetGPSString("LaserAntenna", connectToSat.OpenAntennaPositions.First()));
                            laserAntenna.Connect();
                            ConnectedTo.Add(connectTo);

                            break;
                        }

                        if (ConnectedTo.Count >= desiredConnections)
                            break;
                    }
                }

                private string GetGPSString(string name, Vector3D vector) {
                    return $"GPS:{name}:{vector.X}:{vector.Y}:{vector.Z}:";
                }
            }
        }

    }
}