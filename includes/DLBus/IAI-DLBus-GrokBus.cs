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
    /* Grok5g_v1
        Vector3D Location
        Vector3D Velocity
        int EntityType              // DLBus.DLBusEntityType            enum
        int Relationship            // MyRelationsBetweenPlayerAndBlock keen enum
        string EntityName
        long EntityId

        using EntityUpdateMessage = MyTuple<Vector3D, Vector3D, int, int, string, long>;
    */

    public partial class Program : MyGridProgram
    {
        public class DLBus_GrokBus
        {

            public enum DLBusGrokEntityType
            {
                None,
                Unknown,
                SmallGrid,
                LargeGrid,
                CharacterHuman,
                CharacterOther,
                FloatingObject,
                Asteroid,
                Planet,
                Meteor,
                Missile,
                Detector,
                Other,
            }
            
            public class DLBusGrokEntity
            {
                public long EntityReceivedTime;
                public Vector3D LastKnownLocation;
                public Vector3D LastKnownVelocityMS;
                public DLBusGrokEntityType EntityType;
                public MyRelationsBetweenPlayerAndBlock Relationship;
                public string EntityName;
                public long EntityId;
            }

            public Action<DLBusGrokEntity> OnGrokEntityReceived;

            private const string GROK_BUS_TOPIC = "DL_BUS_GROK_V1";

            private IMyIntergridCommunicationSystem IGC;
            private IMyBroadcastListener grokListener = null;

            public DLBus_GrokBus(IMyIntergridCommunicationSystem IGC)
            {
                this.IGC = IGC;

                RegisterListeners();
            }

            public void SendEntityDetection(DLBusGrokEntity entity)
            {
                SendEntityDetection(
                    entity.LastKnownLocation,
                    entity.LastKnownVelocityMS,
                    entity.EntityType,
                    entity.Relationship,
                    entity.EntityName,
                    entity.EntityId
                );
            }

            public void SendEntityDetection(Vector3D position, Vector3D velocity, DLBusGrokEntityType type, MyRelationsBetweenPlayerAndBlock relation, string name, long entityId)
            {
                var detection = new MyTuple<Vector3D, Vector3D, int, int, string, long>
                (
                    position, velocity,
                    (int)type, (int)relation, name,
                    entityId
                );

                IGC.SendBroadcastMessage(GROK_BUS_TOPIC, detection, TransmissionDistance.TransmissionDistanceMax);
            }

            public void ProcessListeners(long currentTime)
            {
                while (grokListener.HasPendingMessage) {
                    ProcessGrokEntityMessage(grokListener.AcceptMessage(), currentTime);
                }
            }

            private void ProcessGrokEntityMessage(MyIGCMessage message, long currentTime)
            {
                var grokMessage = message.As<MyTuple<Vector3D, Vector3D, int, int, string, long>>();

                var grokEntity = new DLBusGrokEntity()
                {
                    EntityReceivedTime = currentTime,
                    LastKnownLocation = grokMessage.Item1,
                    LastKnownVelocityMS = grokMessage.Item2,
                    EntityType = (DLBusGrokEntityType)grokMessage.Item3,
                    Relationship = (MyRelationsBetweenPlayerAndBlock)grokMessage.Item4,
                    EntityName = grokMessage.Item5,
                    EntityId = grokMessage.Item6,
                };

                OnGrokEntityReceived?.Invoke(grokEntity);
            }

            private void RegisterListeners()
            {
                grokListener = IGC.RegisterBroadcastListener(GROK_BUS_TOPIC);
            }
        }
    }
}

