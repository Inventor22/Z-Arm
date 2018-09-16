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

namespace Z_Arm_Sliders
{
    public partial class Form1 : Form
    {
        const int verticalReach = 310; // mm
        const int armLength = 200; // mm
        const int generation = 1;
        const int xDir = 1;
        const int yDir = 2;
        const int zDir = 3;
        const int axis1Shoulder = 1;
        const int axis2Elbow = 2;
        const int axis3Z = 3;
        const int axis4Wrist = 4;

        private readonly Dictionary<int, string> initialDebugMap = new Dictionary<int, string>()
        {
            { 0, "Communication has not yet been established, this initialization is unsuccessful" },
            { 1, "Initializing" },
            { 2, "Generation parameter error" },
            { 3, "Encoder value error" },
            { 11, "Controlled by the mobile terminal, this initialization is not successful" },
            { 12, "z_travel transmission error" }    
        };
        
        private int wristAngleDeg = 0;
        private int elbowAngleDeg = 0;
        private int shoulderAngleDeg = 0;
        private int zAxisHeight = 0;

        private int verticalSpeed = 100; // mm/s
        private int angularSpeed = 100; // mm/s

        int zArmId = -1;
        ControlBeanEx robot;

        public Form1()
        {
            InitializeComponent();
            SetManualControls(false);
            wristTextbox.Text = wristTrackbar.Value.ToString();
            elbowTextbox.Text = elbowTrackbar.Value.ToString();
            shoulderTextbox.Text = shoulderTrackBar.Value.ToString();
            zAxisTextbox.Text = zAxisTrackbar.Value.ToString();

            verticalSpeed = int.Parse(speedTextbox.Text);
            angularSpeed = int.Parse(angularSpeedTextbox.Text);
        }

        private async void connectButton_Click(object sender, EventArgs e)
        {
            try
            {
                Task task = Task.Factory.StartNew(() => TcpserverEx.net_port_initial());

                await task;

                if (task.IsFaulted)
                {
                    debugTextbox.AppendText("No net port initial\n");
                }
                if (task.IsCanceled)
                {
                    debugTextbox.AppendText("Cancelled\n");
                }
                if (task.IsCompleted)
                {
                    debugTextbox.AppendText("Net port initial completed\n");
                }
                if (task.Exception != null)
                {
                    throw task.Exception;
                }
            }
            catch (Exception ex)
            {
                debugTextbox.AppendText(ex.Message);
                return;
            }
            
            // Wait for net_port_initial to complete
            await Task.Delay(1200);

            debugTextbox.AppendText("Finding Z-Arm. ");

            bool found = false;
            try
            {
                for (int i = 0; i < 256; i++)
                {
                    if (TcpserverEx.card_number_connect(i) == 1)
                    {
                        zArmId = i;
                        found = true;
                        debugTextbox.AppendText($"Id: {i}\n");
                        break;
                    }
                }
            }
            catch (Exception ex)
            {
                debugTextbox.AppendText(ex.Message);
                return;
            }

            if (!found)
            {
                debugTextbox.AppendText("No arm found. Retry.\n");
                return;
            }

            // Connect to the Z-Arm
            robot = TcpserverEx.get_robot(zArmId);

            int retryCount = 0;
            bool isConnected = false;
            while (!isConnected)
            {
                isConnected = await Task.Factory.StartNew(() => robot.is_connected());
                debugTextbox.AppendText($"Attempting to connect to Z-Arm {zArmId}: {isConnected}\n");
                if (++retryCount > 3)
                {
                    break;
                }
                await Task.Delay(1000);
            }

            if (retryCount >= 3)
            {
                debugTextbox.AppendText($"Could not connect. Retry.\n");
                return;
            }

            int ret = -1;
            retryCount = 0;
            while (ret != 1)
            {
                ret = await Task.Factory.StartNew(() => robot.initial(generation, verticalReach));
                debugTextbox.AppendText($"Initializing: {ret} = {initialDebugMap[ret]}\n");
                if (++retryCount > 3)
                {
                    break;
                }
                await Task.Delay(1000);
            }

            if (ret != 1)
            {
                debugTextbox.AppendText($"Could not initialize Z-Arm Id {zArmId}, return value: {ret}\n");
                return;
            }
            else
            {
                robot.unlock_position();
                robot.set_arm_length(armLength, armLength);
                robot.set_catch_or_release_accuracy(0.5f);
                debugTextbox.AppendText($"Z Arm Id {zArmId} Initialized!\n");

                SetManualControls(true);
            }
        }

        private async void homeButton_Click(object sender, EventArgs e)
        {
            SetManualControls(false);

            robot.set_angle_move(0, 0, 0, 0, angularSpeed);

            while (!robot.is_robot_goto_target())
            {
                robot.get_scara_param();

                SetTrackbar(wristTrackbar, wristTextbox, (int)robot.rotation);
                SetTrackbar(elbowTrackbar, elbowTextbox, (int)robot.angle2);
                SetTrackbar(shoulderTrackBar, shoulderTextbox, (int)robot.angle1);
                SetTrackbar(zAxisTrackbar, zAxisTextbox, (int)robot.z);

                await Task.Delay(50);
            }

            //wristTextbox.Text = 0.ToString();
            //elbowTextbox.Text = 0.ToString();
            //shoulderTextbox.Text = 0.ToString();
            //zAxisTextbox.Text = 0.ToString();

            //wristTrackbar.Value = 0;
            //elbowTrackbar.Value = 0;
            //shoulderTrackBar.Value = 0;
            //zAxisTrackbar.Value = 0;

            SetManualControls(true);
        }

        bool wristUpdated = false;
        private void wristTrackbar_ValueChanged(object sender, EventArgs e)
        {
            wristUpdated = true;
        }
        private void wristTrackbar_MouseUp(object sender, MouseEventArgs e)
        {
            if (wristUpdated)
            {
                wristUpdated = false;

                wristAngleDeg = wristTrackbar.Value;

                robot.set_angle_move(shoulderAngleDeg, elbowAngleDeg, zAxisHeight, wristAngleDeg, angularSpeed);
            }
        }

        bool elbowUpdated = false;
        private void elbowTrackbar_ValueChanged(object sender, EventArgs e)
        {
            elbowUpdated = true;
        }
        private void elbowTrackbar_MouseUp(object sender, MouseEventArgs e)
        {
            if (elbowUpdated)
            {
                elbowUpdated = false;

                elbowAngleDeg = elbowTrackbar.Value;

                robot.set_angle_move(shoulderAngleDeg, elbowAngleDeg, zAxisHeight, wristAngleDeg, angularSpeed);
            }
        }

        bool shoulderUpdated = false;
        private void shoulderTrackBar_ValueChanged(object sender, EventArgs e)
        {
            shoulderUpdated = true;
        }
        private void shoulderTrackBar_MouseUp(object sender, MouseEventArgs e)
        {
            if (shoulderUpdated)
            {
                shoulderUpdated = false;
                shoulderAngleDeg = shoulderTrackBar.Value;

                robot.set_angle_move(shoulderAngleDeg, elbowAngleDeg, zAxisHeight, wristAngleDeg, angularSpeed);
            }
        }

        bool zAxisUpdated = false;
        private void zAxisTrackbar_ValueChanged(object sender, EventArgs e)
        {
            zAxisUpdated = true;
        }
        private void zAxisTrackbar_MouseUp(object sender, MouseEventArgs e)
        {
            if (zAxisUpdated)
            {
                zAxisUpdated = false;
                zAxisHeight = zAxisTrackbar.Value;

                robot.set_angle_move(shoulderAngleDeg, elbowAngleDeg, zAxisHeight, wristAngleDeg, verticalSpeed);
            }
        }

        // Scroll textbox updates
        private void wristTrackbar_Scroll(object sender, EventArgs e)
        {
            wristTextbox.Text = wristTrackbar.Value.ToString();
        }

        private void elbowTrackbar_Scroll(object sender, EventArgs e)
        {
            elbowTextbox.Text = elbowTrackbar.Value.ToString();
        }

        private void shoulderTrackBar_Scroll(object sender, EventArgs e)
        {
            shoulderTextbox.Text = shoulderTrackBar.Value.ToString();
        }

        private void zAxisTrackbar_Scroll(object sender, EventArgs e)
        {
            zAxisTextbox.Text = zAxisTrackbar.Value.ToString();
        }

        private void speedTextbox_TextChanged(object sender, EventArgs e)
        {
            int tempSpeed = 0;
            if (int.TryParse(speedTextbox.Text, out tempSpeed))
            {
                verticalSpeed = tempSpeed; 
            }
            else
            {
                string s = speedTextbox.Text;
                if (s.Length <= 1)
                {
                    speedTextbox.Text = string.Empty;
                }
                else
                {
                    speedTextbox.Text = s.Substring(0, s.Length - 1);
                }
            }
        }
        
        private void angularSpeedTextbox_TextChanged(object sender, EventArgs e)
        {
            int tempSpeed = 0;
            if (int.TryParse(angularSpeedTextbox.Text, out tempSpeed))
            {
                angularSpeed = tempSpeed;
            }
            else
            {
                string s = angularSpeedTextbox.Text;
                if (s.Length <= 1)
                {
                    angularSpeedTextbox.Text = string.Empty;
                }
                else
                {
                    angularSpeedTextbox.Text = s.Substring(0, s.Length - 1);
                }
            }
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            try
            {
                TcpserverEx.close_tcpserver();
            }
            catch
            { }
        }

        Task task = null;
        CancellationTokenSource cancellationSource;
        bool looping = false;
        private async void stopButton_Click(object sender, EventArgs e)
        {
            if (!looping)
            {
                if (task != null)
                {
                    // Wait for any existing task to end.
                    await task;
                }

                SetManualControls(false);

                looping = true;
                cancellationSource = new CancellationTokenSource();

                debugTextbox.AppendText("Looping started");

                task = Task.Factory.StartNew(async (cancellationToken) =>
                {
                    CancellationToken token = (CancellationToken)cancellationToken;

                    int[][] coords = new int[][]
                    {
                        new int[] { -80,  90,    0, wristAngleDeg },
                        new int[] {  80, -90, -100, wristAngleDeg },
                        new int[] { -80,  90, -200, wristAngleDeg },
                    };

                    while (true)
                    {
                        foreach (int[] coord in coords)
                        {
                            robot.set_angle_move(coord[0], coord[1], coord[2], coord[3], verticalSpeed);

                            while (!robot.is_robot_goto_target())
                            {
                                if (token.IsCancellationRequested)
                                    return;

                                robot.get_scara_param();

                                SetTrackbar(wristTrackbar, wristTextbox, (int)robot.rotation);
                                SetTrackbar(elbowTrackbar, elbowTextbox, (int)robot.angle2);
                                SetTrackbar(shoulderTrackBar, shoulderTextbox, (int)robot.angle1);
                                SetTrackbar(zAxisTrackbar, zAxisTextbox, (int)robot.z);

                                await Task.Delay(50);
                            }
                        }
                    }
                }, cancellationSource.Token);
            }
            else
            {
                looping = false;
                cancellationSource.Cancel(false);
                debugTextbox.AppendText("Looping stopped");
                SetManualControls(true);
            }
        }

        bool teaching = false;
        CancellationTokenSource cancellationSource2;
        private void teachButton_Click(object sender, EventArgs e)
        {
            if (!teaching)
            {
                SetManualControls(false);

                teaching = true;
                teachButton.Text = "!Teach";
                debugTextbox.AppendText("Teaching mode enabled\n");
                robot.set_drag_teach(true);

                cancellationSource2 = new CancellationTokenSource();

                Task.Factory.StartNew(async (cancellationToken) =>
                {
                    CancellationToken token = (CancellationToken)cancellationToken;

                    int i = 0;
                    while (!token.IsCancellationRequested)
                    {
                        robot.get_scara_param();
                        
                        SetTrackbar(wristTrackbar, wristTextbox, (int) robot.rotation);
                        SetTrackbar(elbowTrackbar, elbowTextbox, (int)robot.angle2);
                        SetTrackbar(shoulderTrackBar, shoulderTextbox, (int)robot.angle1);
                        SetTrackbar(zAxisTrackbar, zAxisTextbox, (int)robot.z);

                        if (i++ % 10 == 0)
                        {
                            AppendTextBox(debugTextbox, $"wrist: {robot.rotation}, elbow: {robot.angle2}, shoulder: {robot.angle1}, Z: {robot.z}\n");
                            Console.WriteLine($"wrist: {robot.rotation}, elbow: {robot.angle2}, shoulder: {robot.angle1}, Z: {robot.z}");
                        }

                        await Task.Delay(100);
                    }

                }, cancellationSource2.Token);
            }
            else
            {
                teaching = false;
                cancellationSource2.Cancel(false);
                teachButton.Text = "Teach";
                debugTextbox.AppendText("Teaching mode disabled\n");
                robot.set_drag_teach(false);
                SetManualControls(true);
            }
        }

        // Only update textboxes on the UI thread.
        private void SetTextBox(TextBox textBox, string value)
        {
            if (InvokeRequired)
            {
                this.Invoke(new Action<TextBox, string>(SetTextBox), new object[] { textBox, value });
                return;
            }
            textBox.Text = value;
        }

        private void AppendTextBox(TextBox textbox, string value)
        {
            if (InvokeRequired)
            {
                this.Invoke(new Action<TextBox, string>(AppendTextBox), new object[] { textbox, value });
                return;
            }
            textbox.AppendText(value);
        }

        private void SetTrackbar(TrackBar trackbar, TextBox textbox, int value)
        {
            if (InvokeRequired)
            {
                this.Invoke(new Action<TrackBar, TextBox, int>(SetTrackbar), new object[] { trackbar, textbox, value });
                return;
            }
            trackbar.Value = Math.Min(Math.Max(trackbar.Minimum, value), trackbar.Maximum);
            textbox.Text = value.ToString();
        }

        private void SetManualControls(bool enabled)
        {
            homeButton.Enabled = enabled;
            wristTrackbar.Enabled = enabled;
            elbowTrackbar.Enabled = enabled;
            zAxisTrackbar.Enabled = enabled;
            shoulderTrackBar.Enabled = enabled;
            wristTextbox.Enabled = enabled;
            elbowTextbox.Enabled = enabled;
            shoulderTextbox.Enabled = enabled;
            zAxisTextbox.Enabled = enabled;
        }
    }
}
