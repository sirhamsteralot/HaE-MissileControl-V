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
            public enum MissileHealth
            {
                Healthy,
                Degraded,
                Dead,
            }

            List<IMyThrust> thrusters = new List<IMyThrust>();
            List<IMyGyro> gyros = new List<IMyGyro>();
            List<IMyWarhead> warheads = new List<IMyWarhead>();
            List<IMyGasTank> gasTanks = new List<IMyGasTank>();
            List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();

            IMyFlightMovementBlock flightMovementBlock;
            IMyOffensiveCombatBlock offensiveCombatBlock;

            public MissileHealth CheckMissileHealth()
            {
                //TODO
                return MissileHealth.Dead;
            }
        }
    }
}