using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using ControlBeanExDll;
using TcpserverExDll;
using UnityEngine;
using UnityEditor;

namespace ZArmUnityAdapter
{
    [InitializeOnLoad]
    public class ZArmAdapter
    {
        static ZArmAdapter() // static Constructor
        {
            var currentPath = Environment.GetEnvironmentVariable("PATH", EnvironmentVariableTarget.Process);

            #if UNITY_EDITOR_32
                var dllPath = Application.dataPath
                    + Path.DirectorySeparatorChar + "Plugins"
                    + Path.DirectorySeparatorChar + "x86";
            #elif UNITY_EDITOR_64
                var dllPath = Application.dataPath
                    + Path.DirectorySeparatorChar + "Plugins"
                    + Path.DirectorySeparatorChar + "x86_64";
            #else // Player
                var dllPath = Application.dataPath
                    + Path.DirectorySeparatorChar + "Plugins";
            #endif

            Debug.Log("Current Path: " + currentPath);
            Debug.Log("DLL Path: " + dllPath);

            foreach (string file in Directory.GetFiles(dllPath))
            {
                Debug.Log(file);
            }

            if (currentPath != null && currentPath.Contains(dllPath) == false)
            {
                Environment.SetEnvironmentVariable("PATH", currentPath + Path.PathSeparator
                    + dllPath, EnvironmentVariableTarget.Process);
            }
        }

        private ControlBeanEx robot;
        private int generation = 1;
        private int verticalReach = 310; // mm

        public void InitializeTcpServer()
        {
            TcpserverEx.net_port_initial();
        }

        public IReadOnlyList<int> GetConnectedZArms()
        {
            List<int> ids = new List<int>();

            for (int i = 0; i < 256; i++)
            {
                if (TcpserverEx.card_number_connect(i) == 1)
                {
                    ids.Add(i);
                }
            }

            return ids;
        }

        public bool ConnectToZArm(int zArmId)
        {
            this.robot = TcpserverEx.get_robot(zArmId);
            return this.robot.is_connected();
        }

        public bool IsConnected()
        {
            return this.robot.is_connected();
        }

        public void Initialize()
        {
            this.robot.initial(generation, verticalReach);
        }

        public void SetAngles(float shoulderAngle, float elbowAngle, float wristAngle, float height, float speed)
        {
            this.robot.set_angle_move(shoulderAngle, elbowAngle, height, wristAngle, speed);
        }
    }
}
