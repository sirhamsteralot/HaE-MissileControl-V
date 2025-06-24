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
    partial class Program
    {
        public class ThrustUtils
        {
            public static double GetForwardThrust(List<IMyThrust>thrusters, Vector3D directionForward)
            {
                double sum = 0;
                foreach (var thrust in thrusters)
                {
                    if (Vector3D.Dot(thrust.WorldMatrix.Backward, directionForward) > 0.999)
                    {
                        sum += thrust.MaxEffectiveThrust;
                    }
                }

                return sum;
            }

            public static void SetThrust(List<IMyThrust> thrusters, Vector3D direction, double percent)
            {
                foreach (var thrust in thrusters)
                {
                    if (Vector3D.Dot(thrust.WorldMatrix.Backward, direction) > 0.99)
                    {
                        if (!thrust.Enabled)
                            thrust.Enabled = true;

                        thrust.ThrustOverridePercentage = (float)percent;
                    }
                }
            }

            public static void SetThrustPercentage(List<IMyThrust> thrusters, float percent)
            {
                foreach (var thruster in thrusters)
                {
                    if (!thruster.Enabled)
                        thruster.Enabled = true;

                    thruster.ThrustOverridePercentage = percent;
                }
            }

            public static void SetThrustPercentage(HashSet<IMyThrust> thrusters, float percent)
            {
                foreach (var thruster in thrusters)
                {
                    if (!thruster.Enabled)
                        thruster.Enabled = true;

                    thruster.ThrustOverridePercentage = percent;
                }
            }

            public static void SetMinimumThrust(List<IMyThrust> thrusters, Vector3D direction, double percent)
            {
                foreach (var thrust in thrusters)
                {
                    if (Vector3D.Dot(thrust.WorldMatrix.Backward, direction) > 0.99)
                    {
                        if (!thrust.Enabled)
                            thrust.Enabled = true;

                        thrust.ThrustOverridePercentage = percent > thrust.ThrustOverridePercentage? (float)percent : thrust.ThrustOverridePercentage;
                    }
                }
            }

            public static void SetThrustBasedDot(List<IMyThrust> thrusters, Vector3D direction, double mulitplier = 1)
            {
                foreach (var thrust in thrusters)
                {
                    if (!thrust.Enabled)
                        thrust.Enabled = true;

                    double thrustpercentage = Vector3D.Dot(thrust.WorldMatrix.Backward, direction);
                    if (thrustpercentage < 0.5)
                    {
                        thrustpercentage = 0;
                    }

                    thrust.ThrustOverridePercentage = (float)(thrustpercentage * mulitplier);
                }
            }

            public static void SetThrustNotInDirection(List<IMyThrust> thrusters, Vector3D direction, double percent)
            {
                foreach (var thrust in thrusters)
                {
                    if (Vector3D.Dot(thrust.WorldMatrix.Backward, direction) < 0.9)
                    {
                        if (!thrust.Enabled)
                            thrust.Enabled = true;

                        thrust.ThrustOverridePercentage = (float)percent;
                    }
                }
            }
        }
    }
}
