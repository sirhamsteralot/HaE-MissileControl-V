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
        public class PIDController
        {
            public double Kp, Ki, Kd;
            private double integral;
            private double previousError;
            private double lastUpdateTime;

            public PIDController(double kp, double ki, double kd)
            {
                Kp = kp;
                Ki = ki;
                Kd = kd;
                Reset();
            }

            public void Reset()
            {
                integral = 0;
                previousError = 0;
                lastUpdateTime = -1;
            }

            public double Compute(double error, double currentTime)
            {
                double deltaTime = 0.016; // Default to 60Hz

                if (lastUpdateTime >= 0)
                    deltaTime = currentTime - lastUpdateTime;

                lastUpdateTime = currentTime;

                integral += error * deltaTime;
                double derivative = (error - previousError) / deltaTime;
                previousError = error;

                return Kp * error + Ki * integral + Kd * derivative;
            }
        }
    }
}
