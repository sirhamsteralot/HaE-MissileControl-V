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
        public static class VectorUtils
        {
            public static void CreateFibonacciSphere(List<Vector3D> sphereDirections, int samples, Random rnd = null)
            {
                double rndNr = 1;

                if (rnd != null)
                    rndNr = rnd.NextDouble() * samples;

                double offset = 2 / samples;
                double increment = 2.39996322972865; //= Math.PI * (3 - Math.Sqrt(5));

                for (int i = 0; i < samples; i++)
                {
                    double y = ((i * offset) - 1) + (offset / 2);
                    double r = Math.Sqrt(1 - (y * y));

                    double phi = ((i + rndNr) % samples) * increment;

                    double x = Math.Cos(phi) * r;
                    double z = Math.Sin(phi) * r;

                    sphereDirections.Add(new Vector3D(x, y, z));
                }
            }

            public static Vector2I GenerateRandomVector2I(Random random)
            {
                int ranX0 = random.Next(0, 174);
                int ranY0 = random.Next(0, 174);

                return new Vector2I(ranX0, ranY0);
            }

            /// project one on two
            public static Vector3D Project(Vector3D one, Vector3D two)
            {
                Vector3D projection = one.Dot(two) / two.LengthSquared() * two;
                return projection;
            }

            /// projects onto a plane
            public static Vector3D ProjectOnPlane(Vector3D planeLeft, Vector3D planeForward, Vector3D direction)
            {
                Vector3D normal = Vector3D.Cross(planeLeft, planeForward);
                normal.Normalize();

                Vector3D projection = direction - Project(direction, normal);

                return projection;
            }

            /// projects onto a plane
            public static Vector3D ProjectOnPlanePerpendiculair(Vector3D planeLeft, Vector3D planeForward, Vector3D direction)
            {
                Vector3D projectionLeft = Project(direction, planeLeft);
                Vector3D projectionForward = Project(direction, planeForward);

                return projectionLeft + projectionForward;
            }

            /// proejcts onto a plane
            public static Vector3D ProjectOnPlane(Vector3D planeNormal, Vector3D direction)
            {
                Vector3D projection = direction - Project(direction, planeNormal);

                return projection;
            }

            /// calculate component of one on two
            public static double GetProjectionScalar(Vector3D one, Vector3D two)
            {
                double dotBetween = one.Dot(two);

                return one.Length() * dotBetween;
            }

            public static double GetCrossComponent(Vector3D one, Vector3D two)
            {
                Vector3D crossed = one.Cross(two);

                return crossed.Length();
            }

            /// mirror a over b
            public static Vector3D Reflect(Vector3D a, Vector3D b, double rejectionFactor = 1)
            {
                Vector3D project_a = Project(a, b);
                Vector3D reject_a = a - project_a;
                Vector3D reflect_a = project_a - reject_a * rejectionFactor;
                return reflect_a;
            }

            public static Vector3D Reject(Vector3D a, Vector3D b)
            {
                Vector3D project_a = Project(a, b);
                Vector3D reject_a = a - project_a;
                return reject_a;
            }

            /// returns angle in radians
            public static double GetAngle(Vector3D One, Vector3D Two)
            {
                return Math.Acos(MathHelper.Clamp(One.Dot(Two) / Math.Sqrt(One.LengthSquared() * Two.LengthSquared()), -1, 1));
            }

            public static double SignedAngle(Vector3D from, Vector3D to, Vector3D axis)
            {
                from = Vector3D.Normalize(from);
                to = Vector3D.Normalize(to);
                double angle = Math.Acos(MathHelper.Clamp(Vector3D.Dot(from, to), -1.0, 1.0));
                double sign = Math.Sign(Vector3D.Dot(axis, Vector3D.Cross(from, to)));
                return angle * sign;
            }


            public static Vector3D TransformPosLocalToWorld(MatrixD worldMatrix, Vector3D localPosition)
            {
                return Vector3D.Transform(localPosition, worldMatrix);
            }

            public static Vector3D TransformPosWorldToLocal(MatrixD worldMatrix, Vector3D worldPosition)
            {
                Vector3D referenceWorldPosition = worldMatrix.Translation;
                Vector3D worldDirection = worldPosition - referenceWorldPosition;
                return Vector3D.TransformNormal(worldDirection, MatrixD.Transpose(worldMatrix));
            }

            public static Vector3D TransformDirLocalToWorld(MatrixD worldMatrix, Vector3D localDirection)
            {
                return Vector3D.TransformNormal(localDirection, worldMatrix);
            }

            public static Vector3D TransformDirWorldToLocal(MatrixD worldMatrix, Vector3D worldDirection)
            {
                return Vector3D.TransformNormal(worldDirection, MatrixD.Transpose(worldMatrix));
            }

            public static bool IsEqual(Vector3D v1, Vector3D v2, double tolerance)
            {
                return (v1 - v2).LengthSquared() < tolerance;
            }

            private static Vector3D Quadratic(Vector3D vector)
            {
                return vector * vector;
            }

            private static Vector3D Sqrt(Vector3D vector)
            {
                return new Vector3D(Math.Sqrt(vector.X), Math.Sqrt(vector.Y), Math.Sqrt(vector.Z));
            }
            
            public static Vector3D Slerp(Vector3D from, Vector3D to, double t)
            {
                // Clamp t to [0,1]
                t = Math.Max(0.0, Math.Min(1.0, t));

                double magFrom = from.Length();
                double magTo   = to.Length();

                // If either vector is zero-length, just lerp
                if (magFrom < 1e-6 || magTo < 1e-6)
                    return Vector3D.Lerp(from, to, t);

                // Normalize directions
                Vector3D dirFrom = from / magFrom;
                Vector3D dirTo   = to   / magTo;

                // Compute cosine of angle between
                double cosTheta = Vector3D.Dot(dirFrom, dirTo);
                // Clamp to avoid NaNs from acos
                cosTheta = Math.Max(-1.0, Math.Min(1.0, cosTheta));

                // If angle is very small, fall back to linear interpolation
                const double epsilon = 1e-6;
                if (1.0 - Math.Abs(cosTheta) < epsilon)
                {
                    // Lerp direction, then normalize
                    Vector3D dir = Vector3D.Lerp(dirFrom, dirTo, t).Normalized();
                    // Lerp magnitudes
                    double mag = magFrom + (magTo - magFrom) * t;
                    return dir * mag;
                }

                double theta    = Math.Acos(cosTheta);
                double sinTheta = Math.Sin(theta);

                // Slerp factors
                double factorFrom = Math.Sin((1 - t) * theta) / sinTheta;
                double factorTo   = Math.Sin(t * theta)       / sinTheta;

                // Interpolate direction
                Vector3D slerpedDir = dirFrom * factorFrom + dirTo * factorTo;

                // Interpolate magnitude linearly
                double slerpedMag = magFrom + (magTo - magFrom) * t;

                return slerpedDir * slerpedMag;
            }
        }
	}
}
