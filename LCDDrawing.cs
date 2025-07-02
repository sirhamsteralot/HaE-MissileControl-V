using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Runtime.CompilerServices;
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

using Vector2 = VRageMath.Vector2;

namespace IngameScript
{
    public partial class Program : MyGridProgram
    {
        public class LCDDrawing
        {
            private const int DisplayMissileCount = 5;
            private const long selectedTargetTimeoutTime = 60 * 10;
            private const int SideMargin = 25;
            private const float LockonTextSize = 1.5f;

            private List<IMyTextPanel> textPanels = new List<IMyTextPanel>();
            private List<IMyCockpit> cockpitSurfaces = new List<IMyCockpit>();
            private List<int> cockpitSurfaceConfiguration = new List<int>();
            private List<Missile> launchedMissiles = new List<Missile>();
            private DLBus.DLBusDetectedEntity currentlySelectedTarget;

            private long counter = 0;

            public void AddInformationSurface(IMyTextPanel textSurface)
            {
                textSurface.ContentType = ContentType.SCRIPT;
                textSurface.Script = "";
                textSurface.ScriptBackgroundColor = Color.Black;
                textPanels.Add(textSurface);
            }

            public void AddInformationSurfaceBlock(IMyCockpit cockpit, int surfaceId)
            {
                var textSurface = cockpit.GetSurface(surfaceId);
                if (textSurface != null)
                {
                    textSurface.ContentType = ContentType.SCRIPT;
                    textSurface.Script = "";
                    textSurface.ScriptBackgroundColor = Color.Black;
                    cockpitSurfaces.Add(cockpit);
                    cockpitSurfaceConfiguration.Add(surfaceId);
                }
            }

            public void UpdateLaunchedMissiles(ICollection<Missile> missiles)
            {
                launchedMissiles.Clear();
                launchedMissiles.AddRange(missiles);
            }

            public void SelectTarget(DLBus.DLBusDetectedEntity selectedTarget)
            {
                currentlySelectedTarget = selectedTarget;
            }

            public void DrawAll(long timestamp)
            {
                foreach (var textPanel in textPanels)
                {
                    if (textPanel.IsFunctional)
                    {
                        DrawInformationSurface(textPanel, timestamp);
                    }
                }

                for (int i = 0; i < cockpitSurfaces.Count; i++)
                {
                    if (cockpitSurfaces[i].IsFunctional)
                    {
                        DrawInformationSurface(cockpitSurfaces[i].GetSurface(cockpitSurfaceConfiguration[i]), timestamp);
                    }
                }
            }

            private void DrawInformationSurface(IMyTextSurface textSurface, long timestamp)
            {
                RectangleF viewPort = new RectangleF(
                    (textSurface.TextureSize - textSurface.SurfaceSize) / 2f,
                    textSurface.SurfaceSize
                );

                MySpriteDrawFrame drawFrame = textSurface.DrawFrame();
                if (counter++ % 500 == 0)
                {
                    drawFrame.Add(new MySprite());
                }

                textSurface.ScriptBackgroundColor = Color.Black;
                
                launchedMissiles.RemoveAll(x => x.Health == Missile.MissileHealth.Dead);
                launchedMissiles.SortNoAlloc((x, y) => x.lifeTimeCounter.CompareTo(y.lifeTimeCounter));

                DrawSelectedTarget(ref drawFrame, viewPort, textSurface.SurfaceSize, currentlySelectedTarget, timestamp); // from 0 - 12%

                for (int i = 0; i < Math.Min(launchedMissiles.Count, 5); i++)
                {
                    DrawMissileState(ref drawFrame, viewPort, textSurface.SurfaceSize, launchedMissiles[i], i); // from 12 - 100%
                }

                drawFrame.Dispose();
            }

            private void DrawSelectedTarget(ref MySpriteDrawFrame drawFrame, RectangleF viewPort, Vector2 surfaceSize, DLBus.DLBusDetectedEntity selectedTarget, long currentTime)
            {
                float scaling = GetScaling(surfaceSize);
                float sideMarginP = 0.02f * viewPort.Width;
                float topMarginP = 0.02f * viewPort.Height;

                Color boxColour = Color.Gray;

                if (selectedTarget != null)
                {
                    if (currentTime - selectedTarget.DetectionReceivedTime > selectedTargetTimeoutTime)
                    {
                        boxColour = Color.Orange;
                    }
                    else
                    {
                        boxColour = Color.Green;
                    }
                }


                Vector2 size = new Vector2(viewPort.Width - sideMarginP * 2, viewPort.Height * 0.1f);
                Vector2 position = new Vector2(viewPort.Center.X, viewPort.Center.Y - viewPort.Height / 2 + size.Y/2 + topMarginP);
                MySprite selectionOuterBox = MySprite.CreateSprite("SquareSimple", position, size);
                selectionOuterBox.Color = boxColour;
                drawFrame.Add(selectionOuterBox);

                string centerText = "none";

                if (selectedTarget != null)
                {
                    var cockpit = cockpitSurfaces.FirstOrDefault(x => !x.Closed);
                    if (cockpit != null)
                    {
                        double distance = Vector3D.Distance(selectedTarget.LastKnownLocation, cockpitSurfaces.FirstOrDefault().GetPosition());

                        centerText = distance.ToString("N2");
                    }
                    else
                    {
                        centerText = "target selected";
                    }
                }

                MySprite textSprite = MySprite.CreateText($"[[ {centerText} ]] ", "White", Color.White, LockonTextSize * scaling);
                textSprite.Position = new Vector2(position.X, position.Y - LockonTextSize * 15 * scaling);
                textSprite.Alignment = TextAlignment.CENTER;
                drawFrame.Add(textSprite);
            }

            Random random = new Random();
            private void DrawMissileState(ref MySpriteDrawFrame drawFrame, RectangleF viewPort, Vector2 surfaceSize, Missile missile, int missileNumber)
            {
                float scaling = GetScaling(surfaceSize);
                float sideMarginP = 0.02f * viewPort.Width;
                float topMarginP = 0.02f * viewPort.Height;

                float centerWidth = viewPort.Width - sideMarginP * 2;

                float MissileLineWidth = centerWidth / DisplayMissileCount;
                float HorizontalCenterPosition = viewPort.Center.X - (viewPort.Width / 2) + sideMarginP + missileNumber * (centerWidth / DisplayMissileCount) + MissileLineWidth/2;
                MissileLineWidth -= sideMarginP * 2;

                // lock status Box
                Color lockonBoxColor;
                string lockonBoxText;

                switch (missile.LockType)
                {
                    case Missile.MissileLockType.External:
                        lockonBoxColor = Color.Blue;
                        lockonBoxText = "Ext";
                        break;
                    case Missile.MissileLockType.Internal:
                        lockonBoxColor = Color.Green;
                        lockonBoxText = "Int";
                        break;
                    default:
                        lockonBoxColor = Color.Gray;
                        lockonBoxText = "No";
                        break;
                }

                float boxYPosition = (viewPort.Center.Y - viewPort.Height / 2) + viewPort.Height * 0.225f + topMarginP; // this spans from 12% to 27%
                Vector2 boxSize = new Vector2(MissileLineWidth, viewPort.Height * 0.15f);
                Vector2 boxPosition = new Vector2(HorizontalCenterPosition, boxYPosition);

                MySprite lockTypeBox = MySprite.CreateSprite("SquareSimple", boxPosition, boxSize);
                lockTypeBox.Color = lockonBoxColor;
                drawFrame.Add(lockTypeBox);

                MySprite textSprite = MySprite.CreateText(lockonBoxText, "White", Color.White, 1f * scaling);
                textSprite.Position = new Vector2(boxPosition.X, boxPosition.Y - 1f * 15 * scaling);
                textSprite.Alignment = TextAlignment.CENTER;
                drawFrame.Add(textSprite);

                // Progress Bar
                float progressBoxYPosition = (viewPort.Center.Y - viewPort.Height / 2) + viewPort.Height * 0.48f + topMarginP;
                Vector2 progressBoxSize = new Vector2(boxSize.X, 0.3f * viewPort.Height);
                Vector2 progressBoxPosition = new Vector2(boxPosition.X, progressBoxYPosition);
                MySprite barBox = MySprite.CreateSprite("SquareSimple", progressBoxPosition, progressBoxSize);
                barBox.Color = Color.LightGray;
                drawFrame.Add(barBox);
            
                float progress = 1f-(float)(Math.Min(missile.CurrentTargetDistance, missile.OriginalLaunchDistance) / Math.Max(missile.OriginalLaunchDistance, 1));
                Vector2 progressBarSize = new Vector2(progressBoxSize.X, progressBoxSize.Y * progress);
                Vector2 progressBarPosition = new Vector2(progressBoxPosition.X, progressBoxPosition.Y + progressBoxSize.Y / 2 - progressBarSize.Y / 2);
                MySprite progressBox = MySprite.CreateSprite("SquareSimple", progressBarPosition, progressBarSize);
                progressBox.Color = Color.Green;
                drawFrame.Add(progressBox);

                // Fuel Bar
                float fuelBoxYPosition = (viewPort.Center.Y - viewPort.Height / 2) + viewPort.Height * 0.8f + topMarginP;
                Vector2 fuelBoxSize = new Vector2(boxSize.X, 0.3f * viewPort.Height);
                Vector2 fuelBoxPosition = new Vector2(boxPosition.X, fuelBoxYPosition);
                MySprite fuelBarBox = MySprite.CreateSprite("SquareSimple", fuelBoxPosition, fuelBoxSize);
                fuelBarBox.Color = Color.LightGray;
                drawFrame.Add(fuelBarBox);

                float fuelprogress = (float)(missile.FuelRemainingFraction);
                Vector2 fuelProgressBarSize = new Vector2(fuelBoxSize.X, fuelBoxSize.Y * fuelprogress);
                Vector2 fuelProgressBarPosition = new Vector2(fuelBoxPosition.X, fuelBoxPosition.Y + fuelBoxSize.Y / 2 - fuelProgressBarSize.Y / 2);
                MySprite fuelProgressBox = MySprite.CreateSprite("SquareSimple", fuelProgressBarPosition, fuelProgressBarSize);
                fuelProgressBox.Color = Color.Brown;
                drawFrame.Add(fuelProgressBox);
            }
            
            private float GetScaling(Vector2 surfaceSize)
            {
                float biggestDimension = surfaceSize.Y;
                if (surfaceSize.X < biggestDimension)
                    biggestDimension = surfaceSize.X;

                return MyMath.Clamp(biggestDimension / 512, 0.5f, 1f);
            }
        }
    }
}