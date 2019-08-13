using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using ControlBeanExDll;
using TcpserverExDll;

namespace Z_Arm_Class
{
    public enum ZArmConfig { LowConfiguration = 1, HighConfiguration = 2 };
    public enum ZArmVerticalTravel { Model210, Model310 };

    public enum ZArmAxis { Shoulder = 1, Elbow = 2, Vertical = 3, Wrist = 4 };

    public enum MoveDirection { X = 1, Y = 2, Z = 3};

    public enum InterpolationMode { Polynomial = 0, Linear = 1 };

    public enum PositionInterpolation { SCurve = 1, TCurve = 2 };

    public enum PositionMoveMode { StraightLine = 1, NonLinear = 2 };

    public class ZArm
    {
        private int verticalReach = 310; // mm
        private const int armLength = 200; // mm
        private int generation = 1;

        private int id;
        ControlBeanEx robot;

        private bool serverStarted;

        public ZArm(int id)
        {
            this.id = id;
            this.serverStarted = false;
        }

        public ZArm(int id, ZArmConfig config, ZArmVerticalTravel travel)
        {
            this.id = id;
            this.generation = (int)config;
            this.verticalReach = travel == ZArmVerticalTravel.Model310 ? 310 : 210;
            this.serverStarted = false;
        }

        public static async Task<int> FindZArmId()
        {
            CancellationTokenSource tcs = new CancellationTokenSource(TimeSpan.FromSeconds(5));
            await Task.Factory.StartNew(() => TcpserverEx.net_port_initial(), tcs.Token);
            await Task.Delay(1200);

            return await Task.Factory.StartNew<int>(() =>
            {
                for (int i = 0; i < 256; i++)
                {
                    if (TcpserverEx.card_number_connect(i) == 1)
                    {
                        return i;
                    }
                }
                return -1;
            },
            tcs.Token);
        }

        public async Task Initialize()
        {
            CancellationTokenSource tcs = new CancellationTokenSource(TimeSpan.FromSeconds(5));

            try
            {
                await Task.Factory.StartNew(() => TcpserverEx.net_port_initial(), tcs.Token);
                await Task.Delay(1200);

                this.serverStarted = true;
            }
            catch (TaskCanceledException)
            {
                throw new TaskCanceledException("TcpserverEx.net_port_initial Failed, task cancelled due to timeout");
            }
            catch (Exception ex)
            {
                throw new Exception("TcpserverEx.net_port_initial Failed", ex);
            }

            this.robot = TcpserverEx.get_robot(this.id);
            int attempts = 0;
            bool isSucccess = false;
            while (attempts++ < 3)
            {
                isSucccess = robot.is_connected();
                if (isSucccess)
                {
                    break;
                }

                await Task.Delay(1000);
            }
            if (!isSucccess)
            {
                throw new Exception("Could not connect to Z-Arm with id: " + this.id);
            }

            isSucccess = false;
            int returnCode = -1;
            while (attempts++ < 3)
            {
                returnCode = robot.initial(generation, verticalReach);
                await Task.Delay(1000);
            }
            if (returnCode != 1)
            {
                throw new Exception($"Could not initialize Z-Arm Id {this.id}, Code: {returnCode}, Message: {this.GetDebugMessage(returnCode)}");
            }

            robot.unlock_position();
            robot.set_arm_length(armLength, armLength);
            robot.set_catch_or_release_accuracy(0.5f);
        }

        public float X
        {
            get; private set;
        }

        public float Y
        {
            get; private set;
        }

        public float Z
        {
            get; private set;
        }

        public float ShoulderAngle
        {
            get; private set;
        }

        public float ElbowAngle
        {
            get; private set;
        }

        public float WristAngle
        {
            get; private set;
        }

        public bool CommunicationEstablished
        {
            get; private set;
        }

        public bool IsInitialized
        {
            get; private set;
        }

        public bool IsServoOn
        {
            get; private set;
        }

        public bool IsMoving
        {
            get; private set;
        }

        public bool IsConnected
        {
            get
            {
                return this.robot.is_connected();
            }
        }

        public void GetState()
        {
            this.robot.get_scara_param();
            this.X = this.robot.x;
            this.Y = this.robot.y;
            this.Z = this.robot.z;
            this.ShoulderAngle = this.robot.angle1;
            this.ElbowAngle = this.robot.angle2;
            this.WristAngle = this.robot.rotation;

            this.CommunicationEstablished = this.robot.communicate_success;
            this.IsInitialized = this.robot.initial_finish;
            this.IsServoOn = this.robot.servo_off_flag;
            this.IsMoving = this.robot.move_flag;
        }

        public void GoHome()
        {
            this.SetAngles(0, 0, 0, 0, this.angularVelocity);
        }

        public bool IsAtTarget
        {
            get
            {
                return this.robot.is_robot_goto_target();
            }
        }

        private float angularVelocity;
        public float AngularVelocity
        {
            get
            {
                return this.angularVelocity;
            }
            set
            {
                this.ExecuteCommand(() => this.robot.change_attitude(value));
                this.angularVelocity = value;
            }
        }

        public void SetAngularVelocity(float mmPerSecond)
        {
            int response = this.robot.change_attitude(mmPerSecond);


        }

        public void MoveSingleAxis(ZArmAxis axis, float deltaDistance)
        {
            this.ExecuteCommand(() => 
                this.robot.single_axis_move((int)axis, deltaDistance));
        }

        public void SetAngles(float shoulderAngle, float elbowAngle, float wristAngle, float verticalHeight, float angularVelocity)
        {
            this.ExecuteCommand(() =>
                this.robot.set_angle_move(shoulderAngle, elbowAngle, verticalHeight, wristAngle, angularVelocity));
        }

        public void SetPosition(float x, float y, float z, float wristAngle, float speed, float acceleration, PositionMoveMode moveMode)
        {
            this.ExecuteCommand(() =>
                this.robot.set_position_move(x, y, z, wristAngle, speed, acceleration, (int)PositionInterpolation.TCurve, (int)moveMode));
        }



        private string GetDebugMessage(int code)
        {
            switch (code)
            {
                case 0: return "Arm is running another instruction";
                case 1: return "Success";
                case 2: return "Speed is <= 0";
                case 3: return "Not Initialized yet; Encoder problem";
                case 4: return "Can't reach by the other attitude ??? (in the MOVEL movement, the intermediate process points go out o fbounds and it cannot arrive, and the robotic arm will stop moving";
                case 5: return "direction parameter error";
                case 6: return "Servo not opened";
                case 7: return "In the MOVEL movement, any intermediate process point cannot arrive by the robotic arm's current attitude(attitude), and the robotic arm will stop moving";
                case 8: return "setting acceleration is less than or equal to zero";
                case 9: return "interpolation mode parameter error";
                case 10: return "move_mode move mode error";
                case 11: return "Mobile terminal is controlling";
                case 12: return "z_travel transmission error";
                case 101: return "Incoming parameter";
                case 102: return "In collision, could not move";
                case 103: return "Joint was reset and needs to be initialized again";
                default: return $"Code '{code}' unknown";
            }
        }

        private void ExecuteCommand(Func<int> command)
        {
            int returnCode = command();
            if (returnCode != 1)
            {
                throw new Exception($"Error executing: {command.Method.Name}, Code: {returnCode}, Message: {GetDebugMessage(returnCode)}");
            }
        }

        /// <summary>
        /// true == 24V input signal detected, false == not connected or no signal
        /// </summary>
        public bool GetDigitalInput(int ioNum)
        {
            int code = this.robot.get_digital_out(ioNum);
            if (code == 2)
            {
                throw new ArgumentException($"Invalid IO num: {ioNum}");
            }
            // 0 -> 24v signal input
            // 1 -> not connected, or no signal input
            return code == 0;
        }

        public bool GetDigitalOutput(int ioNum)
        {
            int code = this.robot.get_digital_out(ioNum);
            if (code == -1)
            {
                throw new ArgumentException($"Invalid IO num: {ioNum}");
            }
            return code == 1;
        }

        public void SetDigitalOutput(int ioNum, bool state)
        {
            if (!this.robot.set_digital_out(ioNum, state))
            {
                throw new ArgumentException($"Invalid IO num: {ioNum}");
            }
        }

        public bool CooperationMode
        {
            get
            {
                return this.robot.get_cooperation_fun_state();
            }

            set
            {
                this.robot.set_cooperation_fun_state(value);
            }
        }

        public bool HasCollided
        {
            get
            {
                return this.robot.is_collision();
            }
        }
    }
}
