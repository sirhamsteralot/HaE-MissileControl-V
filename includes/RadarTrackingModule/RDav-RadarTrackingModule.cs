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
        class RadarTrackingModule
        {
            //===============================================================================================
            //This Is A Pretty Generic Targeting Class, I Have Kept It Relatively CLean And Understandable 
            //At Runtime It Is Fairly Lightweight, But Don't Spam It
            //- needs you to update the tracking info every frame 
            //- will throw nullreference if the blocks are destroyed

            //----------------------------------------------------------------------------

            //Used Instead Of A Tuple (keen ree)
            struct TrackingPoint
            {
                public readonly Vector3D Position;
                public readonly double Timestamp;
                public TrackingPoint(Vector3D position, double timestamp)
                {
                    this.Position = position;
                    this.Timestamp = timestamp;
                }
            }

            //Keeps Record Of The Flight Module For This Missile
            IMyFlightMovementBlock L_FlightBlock;
            IMyOffensiveCombatBlock L_CombatBLock;

            // Store last three (position, timestamp) entries 
            TrackingPoint p1;
            TrackingPoint p2;
            TrackingPoint p3;

            //Counting Positions
            public long CurrentTime;
            public int CurrentTick;
            public int RefreshRate = 19;

            //----------------------------------------------------------------------------

            /// <summary>
            /// Constructor, takes 
            /// </summary>
            /// <param name="LBlock_F">The flight block to use</param>
            /// <param name="LBlockC">The combat block to use</param>
            public RadarTrackingModule(IMyFlightMovementBlock LBlock_F, IMyOffensiveCombatBlock LBlockC)
            {
                //Sets
                L_FlightBlock = LBlock_F;
                L_CombatBLock = LBlockC;

                //AI Move Block Settings Used For Continual Tracking
                L_FlightBlock.Enabled = false;
                L_FlightBlock.MinimalAltitude = 10; //possibly could be larger
                L_FlightBlock.PrecisionMode = false;
                L_FlightBlock.SpeedLimit = 400;
                L_FlightBlock.AlignToPGravity = false;
                L_FlightBlock.CollisionAvoidance = false;
                L_FlightBlock.ApplyAction("ActivateBehavior_On");

                //AI combat block settings
                L_CombatBLock.Enabled = true;
                L_CombatBLock.UpdateTargetInterval = 4;
                L_CombatBLock.SearchEnemyComponent.TargetingLockOptions = VRage.Game.ModAPI.Ingame.MyGridTargetingRelationFiltering.Enemy;
                L_CombatBLock.SelectedAttackPattern = 3; //Sets To Intercept Mode
                L_CombatBLock.SetValue<long>("OffensiveCombatIntercept_GuidanceType", 0); // 1 target prediction, 0 basic
                L_CombatBLock.SetValueBool("OffensiveCombatIntercept_OverrideCollisionAvoidance", true); //Sets To Ignore All Collision Detection
                L_CombatBLock.ApplyAction("ActivateBehavior_On");
                L_CombatBLock.ApplyAction("SetTargetingGroup_Weapons"); //comment this out for public version
                L_CombatBLock.ApplyAction("SetTargetPriority_Largest");
            }

            /// <summary>
            //Call This Before Using Any Of The Properties, Updates Position
            /// </summary>
            public void UpdateTracking(long CurrentPBTime_Ticks)
            {
                //Updates Time
                CurrentTime = CurrentPBTime_Ticks;

                // Retrieves the flight block's basic waypoint
                var currentWaypoint = L_FlightBlock.CurrentWaypoint;

                // If no correction is needed, this may be null
                if (currentWaypoint != null)
                {
                    //Updates PositionLocation If Threshold Exceeded
                    if (CurrentTick > RefreshRate)
                    {
                        var positionwaypoint = currentWaypoint.Matrix.GetRow(3);
                        Vector3D TargetPosition = new Vector3D(positionwaypoint.X, positionwaypoint.Y, positionwaypoint.Z);

                        // Shift historical data
                        p1 = p2;
                        p2 = p3;
                        p3 = new TrackingPoint(TargetPosition, CurrentTime);

                        //Resets Counter
                        CurrentTick = 0;
                    }
                    else
                    {
                        //Increments
                        CurrentTick++;
                    }
                }
            }

            /// <summary>
            /// Gets the most recent acceleration vector based on change in velocity over time.
            /// </summary>
            public Vector3D TargetAcceleration
            {
                get
                {
                    // Unpack position and time values manually
                    Vector3D pos1 = p1.Position;
                    double time1 = p1.Timestamp;

                    Vector3D pos2 = p2.Position;
                    double time2 = p2.Timestamp;

                    Vector3D pos3 = p3.Position;
                    double time3 = p3.Timestamp;

                    // Calculate time differences between entries
                    double dt1 = time2 - time1; // Time between p1 and p2
                    double dt2 = time3 - time2; // Time between p2 and p3

                    // Guard against invalid or zero time differences
                    if (dt1 <= 0 || dt2 <= 0)
                        return Vector3D.Zero;

                    // Calculate velocities at each interval
                    Vector3D v1 = (pos2 - pos1) / (double)dt1; // TargetVelocity between p1 and p2
                    Vector3D v2 = (pos3 - pos2) / (double)dt2; // TargetVelocity between p2 and p3

                    // Use average time interval between the two velocity samples
                    double avgDt = (dt1 + dt2) / 2.0;

                    // Compute and return acceleration as the change in velocity over time
                    return (v2 - v1) / (double)avgDt;
                }
            }

            /// <summary>
            /// Gets the most recent velocity vector.
            /// </summary>
            public Vector3D TargetVelocity
            {
                get
                {
                    // Extract position and time from the stored tracking points
                    Vector3D pos2 = p2.Position;
                    double time2 = p2.Timestamp;

                    Vector3D pos3 = p3.Position;
                    double time3 = p3.Timestamp;

                    //Calculates protecting against zero time errors (would give NaN)
                    double dt = time3 - time2;
                    if (dt <= 0) return Vector3D.Zero;

                    //Returns
                    return (pos3 - pos2) / (double)dt;
                }
            }

            /// <summary>
            /// Predicts the target's position using current velocity and acceleration.
            /// </summary>
            public Vector3D TargetPosition
            {
                get
                {

                    //This Is Emergency Ultra Burn, Use Only In Emergencies As Very Performance Intensive
                    //L_CombatBLock.Enabled = false;
                    //L_CombatBLock.Enabled = true;
                    //var CurrentWaypoint = L_FlightBlock.CurrentWaypoint;
                    //var positionwaypoint = CurrentWaypoint.Matrix.GetRow(3);
                    //return new Vector3D(positionwaypoint.X, positionwaypoint.Y, positionwaypoint.Z);

                    // Extracts Current Position
                    Vector3D lastPosition = p3.Position;
                    double lastTime = p3.Timestamp;

                    //Gets V and A
                    Vector3D velocity = TargetVelocity;
                    Vector3D acceleration = TargetAcceleration;

                    //These Boost Provide Improved Performance For High Accel Targets
                    acceleration = 1.25 * acceleration;
                    velocity = 1.25 * velocity;

                    //Timestep
                    double dt = (double)(CurrentTime - lastTime);

                    //S1 = S0 + UT + 0.5AT^2 (simple suvat equation)
                    return lastPosition + velocity * dt + 0.5f * acceleration * dt * dt;
                }
            }

            //----------------------------------------------------------------------------

            /// <summary>
            /// Tells You If Is Tracking Or Not, If This Is True It Is Actively Seeking
            /// </summary>
            public bool IsTracking
            {
                get
                {
                    return L_CombatBLock.SearchEnemyComponent.FoundEnemyId == null ? false : true;
                }
            }

            /// <summary>
            /// Tells You Tracked Object Name
            /// </summary>
            public string TrackedObjectName
            {
                get
                {
                    //this is 
                    string detailedInfo = L_CombatBLock.DetailedInfo;

                    // Split by new lines
                    var lines = detailedInfo.Split('\n');

                    return lines[0];
                }
            }

            /// <summary>
            /// Checks State Of Blocks Internal
            /// </summary>
            public bool CheckWorking()
            {
                if (L_FlightBlock == null || L_FlightBlock.Closed || !L_FlightBlock.IsWorking)
                {return false; }
                if (L_CombatBLock == null || L_CombatBLock.Closed || !L_CombatBLock.IsWorking)
                {return false; }
                return true;
            }

            /// <summary>
            /// Tells You What Is Setup For Line 1 (largest, smallest, closest)
            /// </summary>
            public string GetLine1Info()
            {
                return L_CombatBLock.TargetPriority + "";
            }

            /// <summary>
            /// Tells You What Is Setup For Line 2 (weapons, thrusters etc)
            /// </summary>
            public string GetLine2Info()
            {
                return L_CombatBLock.SearchEnemyComponent.SubsystemsToDestroy + "";
            }
        }
    }
}