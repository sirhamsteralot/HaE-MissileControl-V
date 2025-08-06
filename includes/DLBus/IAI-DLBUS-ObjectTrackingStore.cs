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
        public partial class DLBus
        {
            public class ObjectTrackingStore
            {
                public const float POSITIONEPSILON = 900f; // 30^2 (because sqrt is expensive and this is cheap)
                public const long TIMEOUTTICKS = 30 * 60;
                public const float POSITIONWEIGHT = 1.0f;
                public const float DIRECTIONWEIGHT = 0.3f;

                public class Cluster
                {
                    public double MaxRadius;
                    public Vector3D Centroid;
                    public Vector3D AverageVelocity;
                    public int clusterID;
                    public long LastUpdateTime;

                    public HashSet<DLBus_GrokBus.DLBusGrokEntity> ContainedDetections = new HashSet<DLBus_GrokBus.DLBusGrokEntity>();

                    public Cluster(long id)
                    {
                        clusterID = (int)id;
                    }

                    public bool UpdateDetection(DLBus_GrokBus.DLBusGrokEntity detection, long currentTime)
                    {
                        if (ContainedDetections.Add(detection))
                        {
                            LastUpdateTime = detection.EntityReceivedTime;
                            return UpdateValues(currentTime);
                        }

                        return true;
                    }

                    public void Merge(Cluster other, long currentTime)
                    {
                        foreach (var detection in other.ContainedDetections)
                        {
                            ContainedDetections.Add(detection);
                        }

                        LastUpdateTime = Math.Max(LastUpdateTime, other.LastUpdateTime);

                        UpdateValues(currentTime);
                    }

                    private bool UpdateValues(long currentTime)
                    {
                        ContainedDetections.RemoveWhere(d => currentTime - d.EntityReceivedTime > TIMEOUTTICKS);

                        if (ContainedDetections.Count == 0)
                        {
                            Centroid = Vector3D.Zero;
                            AverageVelocity = Vector3D.Zero;
                            MaxRadius = 0;
                            return false;
                        }

                        Vector3D sumPositions = Vector3D.Zero;
                        Vector3D weightedVelocitySum = Vector3D.Zero;
                        double totalVelocityWeight = 0.0;
                        double maxDistanceSquared = 0;

                        foreach (var detection in ContainedDetections)
                        {
                            long age = currentTime - detection.EntityReceivedTime;
                            Vector3D predictedLocation = PredictLocation(detection.LastKnownLocation, detection.LastKnownVelocityMS, age);

                            sumPositions += predictedLocation;

                            double weight = Math.Exp(-0.01 * age); // λ = 0.01 — tweak for your time units
                            weightedVelocitySum += detection.LastKnownVelocityMS * weight;
                            totalVelocityWeight += weight;


                            double distanceSquared = Vector3D.DistanceSquared(predictedLocation, Centroid);
                            if (distanceSquared > maxDistanceSquared)
                            {
                                maxDistanceSquared = distanceSquared;
                            }
                        }

                        Centroid = sumPositions / ContainedDetections.Count;
                        AverageVelocity = (totalVelocityWeight > 0) ? (weightedVelocitySum / totalVelocityWeight) : Vector3D.Zero;
                        if (ContainedDetections.Count == 1)
                        {
                            MaxRadius = 0;
                        }
                        else
                        {
                            MaxRadius = Math.Sqrt(maxDistanceSquared);
                        }

                        return true;
                    }

                    private Vector3D PredictLocation(Vector3D originalLocation, Vector3D velocity, long TimeDifference)
                    {
                        return (originalLocation + (velocity * ((1.0 / 60.0) * TimeDifference)));
                    }
                }

                public List<Cluster> clusters = new List<Cluster>();
                public List<DLBus_GrokBus.DLBusGrokEntity> detections = new List<DLBus_GrokBus.DLBusGrokEntity>();
                public Random idGenerator = new Random();

                public void AddDetection(DLBus_GrokBus.DLBusGrokEntity detection, long currentTime)
                {
                    if (clusters.Count < 1)
                    {
                        Cluster firstCluster = new Cluster(detection.EntityId);
                        firstCluster.UpdateDetection(detection, currentTime);
                        clusters.Add(firstCluster);
                        return;
                    }

                    foreach (var cluster in clusters)
                    {
                        if (IsPartOfCluster(cluster, detection, currentTime))
                        {
                            cluster.UpdateDetection(detection, currentTime);
                            return;
                        }
                    }

                    Cluster newCluster = new Cluster(detection.EntityId);
                    newCluster.UpdateDetection(detection, currentTime);
                    clusters.Add(newCluster);
                }

                public void CleanupClusters(long currentTime)
                {
                    // Step 1: Remove old clusters (set to null for in-place cleanup)
                    for (int i = 0; i < clusters.Count; i++)
                    {
                        if (clusters[i].LastUpdateTime < currentTime - TIMEOUTTICKS)
                        {
                            clusters[i] = null;
                        }
                    }

                    // Step 2: Merge clusters while marking merged ones as null
                    for (int i = 0; i < clusters.Count; i++)
                    {
                        var a = clusters[i];
                        if (a == null) continue;

                        for (int j = i + 1; j < clusters.Count; j++)
                        {
                            var b = clusters[j];
                            if (b == null) continue;

                            if (ClustersCanBeMerged(a, b))
                            {
                                a.Merge(b, currentTime);
                                clusters[j] = null; // Mark b as merged
                            }
                        }
                    }

                    // Step 3: Compact the list in place (remove nulls)
                    int writeIndex = 0;
                    for (int readIndex = 0; readIndex < clusters.Count; readIndex++)
                    {
                        if (clusters[readIndex] != null)
                        {
                            clusters[writeIndex++] = clusters[readIndex];
                        }
                    }

                    // Remove the tail
                    if (writeIndex < clusters.Count)
                        clusters.RemoveRange(writeIndex, clusters.Count - writeIndex);
                }

                public HashSet<DLBus_GrokBus.DLBusGrokEntity> GetClustersAsDetections()
                {
                    var result = new HashSet<DLBus_GrokBus.DLBusGrokEntity>();

                    foreach (var cluster in clusters)
                    {
                        DLBus_GrokBus.DLBusGrokEntity entity = new DLBus_GrokBus.DLBusGrokEntity()
                        {
                            EntityReceivedTime = cluster.LastUpdateTime,
                            LastKnownLocation = cluster.Centroid,
                            LastKnownVelocityMS = cluster.AverageVelocity,
                            EntityType = DLBus_GrokBus.DLBusGrokEntityType.None,
                            EntityName = $"Cluster #{cluster.clusterID}",
                            EntityId = cluster.clusterID
                        };

                        result.Add(entity);
                    }

                    return result;
                }

                private bool ClustersCanBeMerged(Cluster clusterA, Cluster clusterB)
                {
                    long timeDifferenceB = clusterA.LastUpdateTime - clusterB.LastUpdateTime;
                    Vector3D predictedLocationB = PredictLocation(clusterB.Centroid, clusterB.AverageVelocity, timeDifferenceB);

                    double positionDist = Vector3D.DistanceSquared(clusterA.Centroid, predictedLocationB);
                    double positionScore = 1f - Math.Min(positionDist / (clusterA.MaxRadius + clusterB.MaxRadius), 1f); // 1 = match, 0 = max diff

                    float directionScore = VelocityDirectionSimilarity(clusterA.AverageVelocity, clusterB.AverageVelocity);

                    double similarity = POSITIONWEIGHT * positionScore + DIRECTIONWEIGHT * directionScore;

                    return similarity >= 0.6f; // Threshold to declare as similar
                }

                private bool IsPartOfCluster(Cluster cluster, DLBus_GrokBus.DLBusGrokEntity detection, long detectionTime)
                {
                    Vector3D predictedLocationCluster = PredictLocation(cluster.Centroid, cluster.AverageVelocity, detectionTime - cluster.LastUpdateTime);

                    double positionDist = Vector3D.DistanceSquared(predictedLocationCluster, detection.LastKnownLocation);
                    double positionScore = 1f - Math.Min(positionDist / (POSITIONEPSILON + cluster.MaxRadius), 1f); // 1 = match, 0 = max diff

                    float directionScore = VelocityDirectionSimilarity(cluster.AverageVelocity, detection.LastKnownVelocityMS);

                    double similarity = POSITIONWEIGHT * positionScore + DIRECTIONWEIGHT * directionScore;

                    return similarity >= 0.6f; // Threshold to declare as similar
                }

                private float VelocityDirectionSimilarity(Vector3 a, Vector3 b)
                {
                    float dot = a.Dot(b);
                    float magProduct = a.Length() * b.Length();
                    if (magProduct == 0) return 0;

                    float cosine = dot / magProduct;
                    return Math.Max(0f, cosine);
                }
                
                private Vector3D PredictLocation(Vector3D originalLocation, Vector3D velocity, long TimeDifference)
                {
                    return (originalLocation + (velocity * ((1.0 / 60.0) * TimeDifference)));
                }
            }
        }
    }
}