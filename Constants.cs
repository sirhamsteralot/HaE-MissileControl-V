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
        const string versionString = "v0.1.0";

        const double runtimeSignificance = 0.005;

        const string INISettingsHeader = "MissileControlSettings";

        const string DATALINK_ARGUMENT_TOPIC = "MISSILECONTROL_V_PB_ARGUMENT";

        private readonly string[] runningIndicator = new string[] { "- - - - -", "- - 0 - -", "- 0 - 0 -", "0 - 0 - 0", "- 0 - 0 -", "- - 0 - -" };

    }
}