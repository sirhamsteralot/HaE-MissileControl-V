using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;
using System;
using VRage.Collections;
using VRage.Game.Components;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ObjectBuilders.Definitions;
using VRage.Game;
using VRageMath;

namespace IngameScript
{
	partial class Program
	{
        public class Scheduler
	    {
            public int runsPerTick;
            public int QueueCount => queue.Count;

            protected Queue<Task> queue = new Queue<Task>();

            public Scheduler(int runsPerTick = 1)
            {
                this.runsPerTick = runsPerTick;
            }

            public bool Main()
            {
                for (int i = 0; i < runsPerTick; i++)
                {
                    if (queue.Count == 0)
                        return false;

                    var current = queue.First();

                    if (!current.MoveNext())
                    {
                        queue.Dequeue().Dispose();
                        current.Callback?.Invoke();
                    }
                }

                return true;
            }

            public void AddTask(Task enumerator)
            {
                queue.Enqueue(enumerator);
            }

            public void AddTask(IEnumerator<bool> enumerator)
            {
                var task = new Task(enumerator);
                AddTask(task);
            }

            public void AddTask(IEnumerator<bool> enumerator, Action callback)
            {
                var task = new Task(enumerator, callback);
                AddTask(task);
            }
        }
	}
}
