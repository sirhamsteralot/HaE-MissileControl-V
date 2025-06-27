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
        static string globalScreamValue = "";

        DLBus dlBus;
        DLBus.ObjectTrackingStore externalTrackingStore;

        Scheduler scheduler = new Scheduler();
        MyIni Ini = new MyIni();

        MissileManager missileManager;

        string scriptExcludeTag = "#ExMissileControlV#";
        string scriptIncludeTag = "#MissileControlV#";
        int cockpitLCD = 0;

        List<IMyTextPanel> textPanels = new List<IMyTextPanel>();
        List<IMyFlightMovementBlock> movementBlocks = new List<IMyFlightMovementBlock>();
        List<IMyOffensiveCombatBlock> combatBlocks = new List<IMyOffensiveCombatBlock>();
        List<IMyShipMergeBlock> mergeBlocks = new List<IMyShipMergeBlock>();
        List<IMyGyro> gyros = new List<IMyGyro>();
        List<IMyThrust> thrusters = new List<IMyThrust>();
        List<IMyWarhead> warheads = new List<IMyWarhead>();
        List<IMyGasTank> gasTanks = new List<IMyGasTank>();
        List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();
        IMyCockpit mainCockpit;

        List<DLBus.DLBusDetectedEntity> newDetectedEntitiesList = new List<DLBus.DLBusDetectedEntity>();

        int update100Counter = 0;
        double averageRuntime = 0;


        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update1 | UpdateFrequency.Update10 | UpdateFrequency.Update100;

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
            Ini.Set(INISettingsHeader, "TextPanelIncludeTag", "#MissileCtrl#");
            Ini.Set(INISettingsHeader, "CockpitLCDSelection", 0);

            Me.CustomData = Ini.ToString();
        }

        private void ImportConfig()
        {
            ParseINI();

            scriptExcludeTag = Ini.Get(INISettingsHeader, "ScriptExcludeTag").ToString();
            cockpitLCD = Ini.Get(INISettingsHeader, "CockpitLCDSelection").ToInt32();
            scriptIncludeTag = Ini.Get(INISettingsHeader, "TextPanelIncludeTag").ToString();
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

                return false;
            });
        }

        private void Initialize()
        {
            dlBus = new DLBus(IGC);
            dlBus.OnEntityDetectedNetwork += OnNetworkEntityDetected;

            if (externalTrackingStore == null)
                externalTrackingStore = new DLBus.ObjectTrackingStore();

            missileManager = new MissileManager(World.SmallShipMaxSpeed);
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

            if ((updateSource & UpdateType.Update10) == UpdateType.Update10)
            {
            }

            if ((updateSource & UpdateType.Update100) == UpdateType.Update100)
            {
                Echo($"IAI MissileControl V {versionString}");
                Echo(runningIndicator[update100Counter % runningIndicator.Length]);
                Echo($"runtime average: {averageRuntime:N4}");

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

                update100Counter++;
            }

            if (!string.IsNullOrEmpty(globalScreamValue))
            {
                Echo(globalScreamValue);
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

            if (normalizedArgument.StartsWith("select "))
            {
                // TODO
            }

            if (normalizedArgument.StartsWith("deselect"))
            {
                // TODO
            }

            if (normalizedArgument.StartsWith("dumblaunch"))
            {
                scheduler.AddTask(missileManager.LaunchMissile(mainCockpit, movementBlocks, combatBlocks, mergeBlocks, gyros, thrusters, warheads, gasTanks, batteries));
            }

            return false;
        }

        private void OnNetworkEntityDetected(DLBus.DLBusDetectedEntity detectedEntity)
        {
            newDetectedEntitiesList.Add(detectedEntity);
            externalTrackingStore.AddDetection(detectedEntity, Runtime.LifetimeTicks);
        }
    }
}
