﻿using Sandbox.Game.EntityComponents;
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
        DebugAPI debug;
        DLBus dlBus;
        DLBus.ObjectTrackingStore externalTrackingStore;

        Scheduler scheduler = new Scheduler();
        MyIni Ini = new MyIni();

        MissileManager missileManager;
        LCDDrawing ui = new LCDDrawing();

        string scriptExcludeTag = "#ExMissileControlV#";
        string scriptIncludeTag = "#MissileControlV#";
        int cockpitLCD = 0;
        double proximityDetonationDistance = 5;
        double proximityArmingDistance = 25;
        double minimumPGain = 4;
        double maximumPGain = 15;

        List<IMyTextPanel> textPanels = new List<IMyTextPanel>();
        List<IMyFlightMovementBlock> movementBlocks = new List<IMyFlightMovementBlock>();
        List<IMyOffensiveCombatBlock> combatBlocks = new List<IMyOffensiveCombatBlock>();
        List<IMyShipMergeBlock> mergeBlocks = new List<IMyShipMergeBlock>();
        List<IMyGyro> gyros = new List<IMyGyro>();
        List<IMyThrust> thrusters = new List<IMyThrust>();
        List<IMyWarhead> warheads = new List<IMyWarhead>();
        List<IMyGasTank> gasTanks = new List<IMyGasTank>();
        List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();
        List<IMyShipConnector> connectors = new List<IMyShipConnector>();
        List<IMySoundBlock> soundBlocks = new List<IMySoundBlock>();
        List<IMyBroadcastController> broadcastControllers = new List<IMyBroadcastController>();
        IMyCockpit mainCockpit;

        List<DLBus.DLBusDetectedEntity> newDetectedEntitiesList = new List<DLBus.DLBusDetectedEntity>();

        int update100Counter = 0;
        double averageRuntime = 0;

        DLBus.DLBusDetectedEntity currentlySelectedEntity = null;
        IMyBroadcastListener missileControlArgumentListener;


        public Program()
        {
            if (DEBUG_VERSION)
                debug = new DebugAPI(this);

            Runtime.UpdateFrequency = UpdateFrequency.Update1 | UpdateFrequency.Update100;

            if (string.IsNullOrWhiteSpace(Me.CustomData))
                ExportConfig();

            ImportConfig();

            FetchBlocks();
            FetchMissileBlocks();

            Initialize();
        }

        #region INI
        private void ParseINI()
        {
            if (!string.IsNullOrWhiteSpace(Me.CustomData))
            {
                MyIniParseResult parseResult;
                if (!Ini.TryParse(Me.CustomData, out parseResult))
                {
                    Echo($"CustomData error:\nLine {parseResult}");
                }
            }
        }

        private void ExportConfig()
        {
            Ini.AddSection(INISettingsHeader);
            Ini.Set(INISettingsHeader, "ScriptExcludeTag", "#ExMissileCtrl#");
            Ini.Set(INISettingsHeader, "ScriptIncludeTag", "#MissileCtrl#");
            Ini.Set(INISettingsHeader, "CockpitLCDSelection", 0);
            Ini.Set(INISettingsHeader, "ProximityDetonationDistance", 5);
            Ini.Set(INISettingsHeader, "ProximityArmingDistance", 25);

            Me.CustomData = Ini.ToString();
        }

        private void ImportConfig()
        {
            ParseINI();

            scriptExcludeTag = Ini.Get(INISettingsHeader, "ScriptExcludeTag").ToString();
            cockpitLCD = Ini.Get(INISettingsHeader, "CockpitLCDSelection").ToInt32();
            scriptIncludeTag = Ini.Get(INISettingsHeader, "ScriptIncludeTag").ToString();
            proximityDetonationDistance = Ini.Get(INISettingsHeader, "ProximityDetonationDistance").ToDouble();
            proximityArmingDistance = Ini.Get(INISettingsHeader, "ProximityArmingDistance").ToDouble();
        }
        #endregion

        private void FetchBlocks()
        {
            GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(null, x =>
            {
                if (x.CustomData.Contains(scriptExcludeTag))
                    return false;

                if (x.CustomName.Contains(scriptExcludeTag))
                    return false;

                IMyTextPanel textPanel = x as IMyTextPanel;
                if (textPanel != null)
                {
                    if (textPanel.CustomName.Contains(scriptIncludeTag) ||
                        textPanel.CustomData.Contains(scriptIncludeTag))
                    {
                        textPanels.Add(textPanel);
                    }

                    return false;
                }

                var cockpit = x as IMyCockpit;
                if (cockpit != null)
                {
                    if (mainCockpit == null || cockpit.IsMainCockpit)
                    {
                        mainCockpit = cockpit;
                        return false;
                    }
                }

                return false;
            });
        }

        private void FetchMissileBlocks()
        {
            movementBlocks.Clear();
            combatBlocks.Clear();
            mergeBlocks.Clear();
            connectors.Clear();
            gyros.Clear();
            thrusters.Clear();
            gasTanks.Clear();
            batteries.Clear();
            warheads.Clear();
            broadcastControllers.Clear();
            soundBlocks.Clear();

            GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(null, x =>
            {
                if (x.CustomData.Contains(scriptExcludeTag))
                    return false;

                if (x.CustomName.Contains(scriptExcludeTag))
                    return false;

                var movementBlock = x as IMyFlightMovementBlock;
                if (movementBlock != null)
                {
                    movementBlocks.Add(movementBlock);
                    return false;
                }

                var combatBlock = x as IMyOffensiveCombatBlock;
                if (combatBlock != null)
                {
                    combatBlocks.Add(combatBlock);
                    return false;
                }

                var warhead = x as IMyWarhead;
                if (warhead != null)
                {
                    warheads.Add(warhead);
                    return false;
                }

                var mergeBlock = x as IMyShipMergeBlock;
                if (mergeBlock != null)
                {
                    if (mergeBlock.CustomName.Contains(scriptIncludeTag) ||
                        mergeBlock.CustomData.Contains(scriptIncludeTag))
                    {
                        mergeBlocks.Add(mergeBlock);
                    }
                    return false;    

                }

                var connector = x as IMyShipConnector;
                if (connector != null)
                {
                    connectors.Add(connector);
                    return false;
                }

                var gyro = x as IMyGyro;
                if (gyro != null)
                {
                    gyros.Add(gyro);
                    return false;
                }

                var thruster = x as IMyThrust;
                if (thruster != null)
                {
                    thrusters.Add(thruster);
                    return false;
                }

                var gasTank = x as IMyGasTank;
                if (gasTank != null)
                {
                    gasTanks.Add(gasTank);
                    return false;
                }

                var battery = x as IMyBatteryBlock;
                if (battery != null)
                {
                    batteries.Add(battery);
                    return false;
                }

                var soundblock = x as IMySoundBlock;
                if (soundblock != null)
                {
                    soundBlocks.Add(soundblock);
                }

                var broadcastController = x as IMyBroadcastController;
                if (broadcastController != null)
                {
                    broadcastControllers.Add(broadcastController);
                }

                return false;
            });
        }

        private void Initialize()
        {
            dlBus = new DLBus(IGC);
            dlBus.OnEntityDetectedNetwork += OnNetworkEntityDetected;

            if (externalTrackingStore == null)
                externalTrackingStore = new DLBus.ObjectTrackingStore();

            missileManager = new MissileManager(World.SmallShipMaxSpeed, proximityArmingDistance, proximityDetonationDistance, debug);

            foreach (var textPanel in textPanels)
            {
                ui.AddInformationSurface(textPanel);
            }

            if (mainCockpit != null)
                ui.AddInformationSurfaceBlock(mainCockpit, cockpitLCD);
                
            missileControlArgumentListener = IGC.RegisterBroadcastListener(MISSILECONTROL_ARGUMENT_TOPIC);
        }

        public void Main(string argument, UpdateType updateSource)
        {
            averageRuntime = averageRuntime * (1 - runtimeSignificance) + Runtime.LastRunTimeMs * runtimeSignificance;

            HandleUserArguments(argument);

            if ((updateSource & UpdateType.Update1) == UpdateType.Update1)
            {
                dlBus.ProcessListeners(Runtime.LifetimeTicks);
                missileManager.ManageMissiles(Runtime.LifetimeTicks);
                missileManager.UpdateDetectionsDeferred(newDetectedEntitiesList);
                newDetectedEntitiesList.Clear();

                scheduler.Main();
            }

            if ((updateSource & UpdateType.Update100) == UpdateType.Update100)
            {
                Echo($"IAI MissileControl V {versionString}");
                if (DEBUG_VERSION)
                    Echo("DEBUG ENABLED!");
                Echo(runningIndicator[update100Counter % runningIndicator.Length]);
                Echo($"runtime average: {averageRuntime:N4}");

                ui.UpdateLaunchedMissiles(missileManager.launchedMissiles);
                ui.SelectTarget(currentlySelectedEntity);
                ui.DrawAll(Runtime.LifetimeTicks);

                if (mainCockpit != null && mainCockpit.IsFunctional)
                {
                    Vector3D planetPosition;
                    if (mainCockpit.TryGetPlanetPosition(out planetPosition))
                    {
                        double elevation;
                        if (mainCockpit.TryGetPlanetElevation(MyPlanetElevation.Sealevel, out elevation))
                        {
                            double sealevelRadius = Vector3D.Distance(mainCockpit.GetPosition(), planetPosition) - elevation;
                            missileManager.UpdatePlanetValues(planetPosition, mainCockpit.GetNaturalGravity().Length(), sealevelRadius);
                        }
                    }
                }

                if (update100Counter % 5 == 0)
                {
                    FetchMissileBlocks();
                }

                externalTrackingStore.CleanupClusters(Runtime.LifetimeTicks);

                update100Counter++;
            }
        }

        public void HandleIGCMessages()
        {
            while (missileControlArgumentListener.HasPendingMessage)
            {
                var message = missileControlArgumentListener.AcceptMessage();

                var argument = message.As<string>();

                HandleUserArguments(argument);
            }
        }

        public bool HandleUserArguments(string argument)
        {
            if (argument == string.Empty)
            {
                return false;
            }

            string normalizedArgument = argument.ToLower().Trim();
            if (normalizedArgument == "export")
            {
                ExportConfig();
                return true;
            }

            if (normalizedArgument.StartsWith("select"))
            {
                var detections = externalTrackingStore.GetClustersAsDetections();

                DLBus.DLBusDetectedEntity closestMatch = null;
                double closestDotDir = -1;

                foreach (var detection in detections)
                {
                    if (closestMatch == null)
                    {
                        closestMatch = detection;
                    }

                    Vector3D detectionDir = detection.LastKnownLocation - mainCockpit.GetPosition();
                    double distance = detectionDir.Normalize();

                    double dotDirection = Vector3D.Dot(mainCockpit.WorldMatrix.Forward, detectionDir);

                    if (dotDirection > closestDotDir)
                    {
                        closestMatch = detection;
                        closestDotDir = dotDirection;
                    }
                }

                if (closestMatch != null)
                {
                    currentlySelectedEntity = closestMatch;
                    string selectMessage = $"select {currentlySelectedEntity.EntityId}";
                    Echo(selectMessage);
                    IGC.SendBroadcastMessage(DATALINK_ARGUMENT_TOPIC, selectMessage, TransmissionDistance.CurrentConstruct);
                }
                else
                {
                    Echo("no entities!");
                }


            }

            if (normalizedArgument.StartsWith("deselect"))
            {
                currentlySelectedEntity = null;
                string deselectMessage = $"deselect";
                IGC.SendBroadcastMessage(DATALINK_ARGUMENT_TOPIC, deselectMessage);
            }

            if (normalizedArgument.StartsWith("launch"))
            {
                if (currentlySelectedEntity != null)
                {
                    scheduler.AddTask(missileManager.LaunchMissile(mainCockpit, movementBlocks, combatBlocks, mergeBlocks, gyros, thrusters, warheads, gasTanks, batteries, connectors, broadcastControllers, soundBlocks, currentlySelectedEntity));
                    Echo($"Launched at {currentlySelectedEntity.LastKnownLocation}");
                }
                else
                {
                    Echo($"No Target selected!");
                }

            }

            if (normalizedArgument.StartsWith("dumblaunch"))
            {
                scheduler.AddTask(missileManager.LaunchMissile(mainCockpit, movementBlocks, combatBlocks, mergeBlocks, gyros, thrusters, warheads, gasTanks, batteries, connectors, broadcastControllers, soundBlocks));
            }

            return false;
        }

        private void OnNetworkEntityDetected(DLBus.DLBusDetectedEntity detectedEntity)
        {
            if (currentlySelectedEntity != null && currentlySelectedEntity.EntityId == detectedEntity.EntityId)
                currentlySelectedEntity = detectedEntity;

            newDetectedEntitiesList.Add(detectedEntity);
            externalTrackingStore.AddDetection(detectedEntity, Runtime.LifetimeTicks);
        }
    }
}
