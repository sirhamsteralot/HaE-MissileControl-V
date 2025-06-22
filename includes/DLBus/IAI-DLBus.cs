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
    /*
        long LocalTime
        Vector3D Location
        Vector3D velocity

        using StatusMessage = MyTuple<long, Vector3D, Vector3D>;
    */

    /*
        Vector3D Location
        Vector3D Velocity
        double Size
        int DetectedEntityType
        string Name
        long EntityId
        int Relationship

        using DetectionMessage = MyTuple<Vector3D, Vector3D, double, int, string, long, int>;
    */

    /*
        Vector3D Position
        List<Vector3D> OpenAntennaPositions
        List<long> ConnectedTo

        using CommsSatAdvertisement = MyTuple<Vector3D, ImmutableList<Vector3D>, ImmutableList<long>>;
    */

    public partial class Program : MyGridProgram
    {
        public partial class DLBus
        {
            public class DLBusEndPoint
            {
                // this is the offset of the time to the one that the remote endpoint uses. 
                // All other time handling is done in our pb's localtime
                public long LocalTimeOffset;
                public long LastUpdateTime;
                public Vector3D LastKnownLocation;
                public Vector3D LastKnownVelocity;

                public long GetTime(long currentTime)
                {
                    return currentTime + LocalTimeOffset;
                }
            }

            public class DLBusDetectedEntity
            {
                public long DetectionReceivedTime;
                public Vector3D LastKnownLocation;
                public Vector3D LastKnownVelocity;
                public double Size;
                public MyDetectedEntityType detectedEntityType;
                public string Name;
                public long EntityId;

                public override bool Equals(object obj)
                {
                    DLBusDetectedEntity other = obj as DLBusDetectedEntity;
                    if (obj != null)
                    {
                        return this.EntityId == other.EntityId;
                    }
                    return false;
                }

                public override int GetHashCode()
                {
                    return EntityId.GetHashCode();
                }
            }

            public Action<DLBusDetectedEntity> OnEntityDetectedNetwork;
            public Action<long> OnCommsSatteliteAdvertisement;

            private const string DETECTION_TOPIC = "DL_BUS_Detection";
            private const string STATUS_TOPIC = "DL_BUS_Status";
            private const string COMMS_SAT_ADVERTISEMENT_TOPIC = "DL_BUS_Satellite_Advertisement";

            private IMyIntergridCommunicationSystem IGC;
            private Dictionary<string, IMyBroadcastListener> listeners = new Dictionary<string, IMyBroadcastListener>();

            public  Dictionary<long, DLBusEndPoint> registeredEndPoints = new Dictionary<long, DLBusEndPoint>();

            private DLBusLaserCommuncationClient laserCommuncationClient;

            public DLBus(IMyIntergridCommunicationSystem IGC)
            {
                this.IGC = IGC;

                RegisterListeners();
            }

            public List<DLBusEndPoint> GetEndPoints()
            {
                return registeredEndPoints.Values.ToList();
            }

            public int GetActiveLaserConnectionCount()
            {
                return laserCommuncationClient.ActiveConnectionCount();
            }


            public void SendUpdateStatus(long currentTime, Vector3D currentPosition, Vector3D currentVelocity)
            {
                var updateStatus = new MyTuple<long, Vector3D, Vector3D>(currentTime, currentPosition, currentVelocity);

                IGC.SendBroadcastMessage(STATUS_TOPIC, updateStatus, TransmissionDistance.AntennaRelay);
            }

            public void SendEntityDetection(DLBusDetectedEntity entity)
            {
                SendEntityDetection(
                    entity.LastKnownLocation,
                    entity.LastKnownVelocity,
                    entity.Size,
                    entity.detectedEntityType,
                    entity.Name,
                    entity.EntityId
                );
            }

            public void SendEntityDetection(Vector3D position, Vector3D velocity, double size, MyDetectedEntityType type, string name, long entityId)
            {
                var detection = new MyTuple<Vector3D, Vector3D, double, int, string, long>
                (
                    position, velocity, size,
                    (int)type, name,
                    entityId
                );

                IGC.SendBroadcastMessage(DETECTION_TOPIC, detection, TransmissionDistance.TransmissionDistanceMax);
            }

            public void AdvertiseAvailableLaserAntennas(ICollection<IMyLaserAntenna> antennas, Vector3D position)
            {
                List<Vector3D> antennaPositions = new List<Vector3D>();
                foreach (var antenna in antennas)
                {
                    if (antenna.Status == MyLaserAntennaStatus.Idle && antenna.IsFunctional)
                        antennaPositions.Add(antenna.GetPosition());
                }

                ImmutableList<Vector3D> positionList = ImmutableList.CreateRange(antennaPositions);
                ImmutableList<long> connectedToList = ImmutableList.CreateRange(laserCommuncationClient.ConnectedTo);

                var advertisement = new MyTuple<Vector3D, ImmutableList<Vector3D>, ImmutableList<long>>(position, positionList, connectedToList);

                IGC.SendBroadcastMessage(COMMS_SAT_ADVERTISEMENT_TOPIC, advertisement, TransmissionDistance.TransmissionDistanceMax);
            }

            public void SendEntityDetection(MyDetectedEntityInfo detectedEntityInfo)
            {
                SendEntityDetection(
                    detectedEntityInfo.Position,
                    detectedEntityInfo.Velocity,
                    detectedEntityInfo.BoundingBox.Size.Length(),
                    detectedEntityInfo.Type,
                    detectedEntityInfo.Name,
                    detectedEntityInfo.EntityId
                );
            }

            public void ManageLaserCommunicationsClient(List<IMyLaserAntenna> laserAntennas, Vector3D myPosition, long myId)
            {
                laserCommuncationClient = new DLBusLaserCommuncationClient(laserAntennas, myId, myPosition);
            }

            public void RefreshLaserCommunciationsClient(long time)
            {
                laserCommuncationClient?.RefreshConnection(time);
            }

            public void ProcessListeners(long currentTime)
            {
                while (listeners[STATUS_TOPIC].HasPendingMessage)
                {
                    ProcessStatusMessage(listeners[STATUS_TOPIC].AcceptMessage(), currentTime);
                }

                while (listeners[DETECTION_TOPIC].HasPendingMessage)
                {
                    ProcessDetectionMessage(listeners[DETECTION_TOPIC].AcceptMessage(), currentTime);
                }

                while (listeners[COMMS_SAT_ADVERTISEMENT_TOPIC].HasPendingMessage)
                {
                    ProcessCommsSatAdvertisementMessage(listeners[COMMS_SAT_ADVERTISEMENT_TOPIC].AcceptMessage(), currentTime);
                }
            }

            private void ProcessStatusMessage(MyIGCMessage message, long currentTime)
            {
                var status = message.As<MyTuple<long, Vector3D, Vector3D>>();

                if (!registeredEndPoints.ContainsKey(message.Source))
                    registeredEndPoints[message.Source] = new DLBusEndPoint();

                var endpoint = registeredEndPoints[message.Source];
                endpoint.LocalTimeOffset = status.Item1 - currentTime;
                endpoint.LastUpdateTime = currentTime;
                endpoint.LastKnownLocation = status.Item2;
                endpoint.LastKnownVelocity = status.Item3;
            }

            private void ProcessDetectionMessage(MyIGCMessage message, long currentTime)
            {
                var detection = message.As<MyTuple<Vector3D, Vector3D, double, int, string, long>>();

                var detectedEntity = new DLBusDetectedEntity()
                {
                    DetectionReceivedTime = currentTime,
                    LastKnownLocation = detection.Item1,
                    LastKnownVelocity = detection.Item2,
                    Size = detection.Item3,
                    detectedEntityType = (MyDetectedEntityType)detection.Item4,
                    Name = detection.Item5,
                    EntityId = detection.Item6,
                };

                if (registeredEndPoints.ContainsKey(message.Source))
                    registeredEndPoints[message.Source].LastUpdateTime = currentTime;

                OnEntityDetectedNetwork?.Invoke(detectedEntity);
            }

            private void ProcessCommsSatAdvertisementMessage(MyIGCMessage message, long currentTime)
            {
                if (laserCommuncationClient == null)
                    return;

                var advertisement = message.As<MyTuple<Vector3D, ImmutableList<Vector3D>, ImmutableList<long>>>();

                var satAdvertisement = new DLBusLaserCommuncationClient.CommsSatelite()
                {
                    SateliteId = message.Source,
                    LastUpdateTime = currentTime,
                    Position = advertisement.Item1,
                    OpenAntennaPositions = advertisement.Item2.ToList(),
                    ConnectedTo = advertisement.Item3.ToList(),
                };

                laserCommuncationClient.ReceiveAdvertisement(message.Source, satAdvertisement);
                OnCommsSatteliteAdvertisement?.Invoke(message.Source);
            }

            private void RegisterListeners()
            {
                foreach (var listener in listeners.Values)
                {
                    IGC.DisableBroadcastListener(listener);
                }

                listeners.Clear();

                listeners.Add(STATUS_TOPIC, IGC.RegisterBroadcastListener(STATUS_TOPIC));
                listeners.Add(DETECTION_TOPIC, IGC.RegisterBroadcastListener(DETECTION_TOPIC));
                listeners.Add(COMMS_SAT_ADVERTISEMENT_TOPIC, IGC.RegisterBroadcastListener(COMMS_SAT_ADVERTISEMENT_TOPIC));
            }
        }
    }
}

