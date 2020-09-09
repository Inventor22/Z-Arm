using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using TcpserverExDll;
using ControlBeanExDll;
using System.Threading;

namespace ZArm_Test
{
    public partial class Form1 : Form
    {
        const int verticalReach = 310; // mm
        const int xDir = 1;
        const int yDir = 2;
        const int zDir = 3;
        const int axis1Shoulder = 1;
        const int axis2Elbow = 2;
        const int axis3Z = 3;
        const int axis4Wrist = 4;

        int zArmId = -1;

        ControlBeanEx robot;

        int delta = 20; // mm
        int speed = 10; // mm/s

        public Form1()
        {
            InitializeComponent();
        }

        private async void idButton_Click(object sender, EventArgs e)
        {
            try
            {
                Task t = Task.Factory.StartNew(() => TcpserverEx.net_port_initial());

                await t;

                if (t.IsFaulted)
                {
                    debugBox.AppendText("No net port initial\n");
                }
                if (t.IsCompleted)
                {
                    debugBox.AppendText("Net port initial completed\n");
                }
                if (t.IsCanceled)
                {
                    debugBox.AppendText("Cancelled");
                }
                if (t.Exception != null)
                {
                    throw t.Exception;
                }
            }
            catch (Exception ex)
            {
                debugBox.AppendText(ex.Message);
            }

            Thread.Sleep(1000);
            
            bool found = false;
            try
            {
                for (int i = 0; i < 256; i++)
                {
                    if (TcpserverEx.card_number_connect(i) == 1)
                    {
                        zArmId = i;
                        found = true;
                        debugBox.AppendText(string.Format("Zarm id: {0}\n", i));
                        break;
                    }
                }
            }
            catch (Exception ex)
            {
                debugBox.AppendText(ex.Message);
            }

            if (!found)
            {
                debugBox.AppendText("No arm found\n");
            }
        }

        private async void buttonConnect_Click(object sender, EventArgs e)
        {
            robot = TcpserverEx.get_robot(zArmId);
            bool isConnected = false;
            while (!isConnected)
            {
                isConnected = await Task.Factory.StartNew(() => robot.is_connected());
                debugBox.AppendText(string.Format("Attempting to connect to Z-Arm {0}... {1}\n", zArmId, isConnected));
                await Task.Delay(1000);
            }
            
            debugBox.AppendText(string.Format("IsConnected: {0}\n", robot.is_connected()));

            //debugBox.AppendText($"initial_finish: {robot.initial_finish}, communicate_success: {robot.communicate_success}\n");

            int ret = -1;

            while (ret != 1)
            {
                ret = await Task.Factory.StartNew(() => robot.initial(1, 310));
                debugBox.AppendText($"ret: {ret}\n");
                await Task.Delay(1000);
            }

            if (ret != 1)
            {
                debugBox.AppendText(string.Format("Could not initialize Z Arm id {0}, return value: {1}\n", zArmId, ret));
            }
            else
            {
                robot.unlock_position();
                robot.set_arm_length(200, 200);
                robot.set_catch_or_release_accuracy(0.5f);
                debugBox.AppendText(string.Format("Z Arm id: {0} connected!\n", zArmId));                
            }
        }

        private void zUpButton_Click(object sender, EventArgs e)
        {
            robot.xyz_move(zDir, delta, speed);
        }

        private void zDownButton_Click(object sender, EventArgs e)
        {
            robot.xyz_move(zDir, -delta, speed);
        }

        private void shoulderCcwButton_Click(object sender, EventArgs e)
        {
            robot.single_joint_move(axis1Shoulder, delta, speed);
        }

        private void shoulderCwButton_Click(object sender, EventArgs e)
        {
            robot.single_joint_move(axis1Shoulder, -delta, speed);
        }

        private void elbowCcwButton_Click(object sender, EventArgs e)
        {
            robot.single_joint_move(axis2Elbow, delta, speed);
        }

        private void elbowCwButton_Click(object sender, EventArgs e)
        {
            robot.single_joint_move(axis2Elbow, -delta, speed);
        }

        private void wristCcwButton_Click(object sender, EventArgs e)
        {
            robot.single_joint_move(axis4Wrist, delta, speed);
        }

        private void wristCwButton_Click(object sender, EventArgs e)
        {
            robot.single_joint_move(axis4Wrist, -delta, speed);
        }

        private void deltaTextbox_TextChanged(object sender, EventArgs e)
        {
            this.delta = int.Parse(this.deltaTextbox.Text);
        }

        private void speedTextbox_TextChanged(object sender, EventArgs e)
        {
            this.speed = int.Parse(this.speedTextbox.Text);
        }

        private void Form1_Load(object sender, EventArgs e)
        {

        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            TcpserverEx.close_tcpserver();
        }
    }
}
