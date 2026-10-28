using ControlBeanExDll;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using TcpserverExDll;
using Z_Arm_Class;

namespace ZArm_Class_Framework
{
    class Program
    {
        static void Main(string[] args)
        {
            ZArm robot2 = new ZArm(59);
            robot2.Initialize(false).Wait();

            robot2.SetAngles(20, 20, 20, -20, 20);
            while(robot2.IsMoving)
            {
                Console.Write(".");
                Thread.Sleep(50);
            }
            robot2.SetAngles(40, 40, 40, -50, 50);

            //robot2.SetPosition(0, 0, 50, 0, 20, 20, PositionMoveMode.NonLinear);

            Console.ReadKey();
            TcpserverEx.close_tcpserver();
        }

        public static async void DoTheThing()
        {
            Console.WriteLine("Hello ZArm!");

            ZArm robot = new ZArm(59);

            try
            {
                Console.WriteLine("Initializing");
                robot.Initialize().Wait();

                Console.WriteLine("Set Angular Velocity");
                while (!robot.IsAtTarget)
                {
                    Thread.Sleep(1);
                }

                robot.AngularVelocity = 50;

                Console.WriteLine("Going Home");
                await robot.GoHome(TimeSpan.FromSeconds(15));
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
        }

        public static void mach()
        {
            Console.WriteLine("Init");

            TcpserverEx.net_port_initial();

            ControlBeanEx robot = TcpserverEx.get_robot(59);

            Stopwatch watch = Stopwatch.StartNew();
            while (!robot.is_connected())
            {
                Console.Write(".");
                Thread.Sleep(1);
            }
            Console.WriteLine();
            Console.WriteLine($"Took {watch.ElapsedMilliseconds}ms to connect!");

            //watch.Restart();
            Console.WriteLine("Joint home 1: " + robot.joint_home(1));
            Console.WriteLine("Joint home 2: " + robot.joint_home(2));
            Console.WriteLine("Joint home 3: " + robot.joint_home(3));
            Console.WriteLine("Joint home 4: " + robot.joint_home(4));

            //Console.ReadKey();
            //Console.WriteLine($"Time to Home: {watch.ElapsedMilliseconds}");
            //Console.ReadKey();

            //while (true)
            //{
            //    if (watch.ElapsedMilliseconds > 15000) break;
            //    robot.get_scara_param();
            //    Console.WriteLine(
            //        $"target: {robot.is_robot_goto_target()}, " +
            //        $"move flag: {robot.move_flag}, " +
            //        $"Angle 1: {robot.angle1}, " +
            //        $"Angle 1 Judge: {robot.angle1_after_judge}, " +
            //        $"Angle 2: {robot.angle2}, " +
            //        $"Angle 2 Judge: {robot.angle2_after_judge}, " +
            //        $"Init Finished: {robot.initial_finish}");
            //    Thread.Sleep(100);
            //}

            Thread.Sleep(3000);

            // robot.set_arm_length(200, 200); DO NOT SET ARM LENGTH WHEN JOINT_HOME IS RUNNING!!!
            Console.WriteLine("Initialize: " + robot.initial(1, 310));
            robot.get_scara_param();
            Console.WriteLine(robot.initial_finish);

            Console.WriteLine("Unlock Position: " + robot.unlock_position());


            watch.Restart();
            while (true)
            {
                if (watch.ElapsedMilliseconds > 15000) break;
                robot.get_scara_param();
                Console.WriteLine(
                    //$"target: {robot.is_robot_goto_target()}, " +
                    $"move flag: {robot.move_flag}, " +
                    $"Angle 1: {robot.angle1}, " +
                    $"Angle 1 Judge: {robot.angle1_after_judge}, " +
                    $"Angle 2: {robot.angle2}, " +
                    $"Angle 2 Judge: {robot.angle2_after_judge}, " +
                    $"Angle 3: {robot.rotation}, " +
                    $"Init Finished: {robot.initial_finish}");
                Thread.Sleep(100);
            }

            robot.set_catch_or_release_accuracy(0.5f);

            //Console.WriteLine("At Home");

            //Console.WriteLine("Going Home");
            //while (robot.move_flag)
            //{
            //    Console.Write(".");
            //    Thread.Sleep(1);
            //}

            //robot.set_arm_length(200, 200);
            //robot.set_catch_or_release_accuracy(0.5f);
            //Console.WriteLine("Unlock Position: " + robot.unlock_position());

            //Console.WriteLine("xyz move: " + robot.xyz_move(1, -10, 50));

            // Console.WriteLine("Cooperation State: " + robot.set_cooperation_fun_state(true));
            //Console.WriteLine("Drag Teach: " + robot.set_drag_teach(true));

        }

        public static void blerp()
        {

            Console.ReadKey();

            Console.WriteLine("Init");

            TcpserverEx.net_port_initial();

            ControlBeanEx robot = TcpserverEx.get_robot(59);

            Stopwatch watch = Stopwatch.StartNew();
            while (!robot.is_connected())
            {
                Console.Write(".");
                Thread.Sleep(1);
            }
            Console.WriteLine();
            Console.WriteLine($"Took {watch.ElapsedMilliseconds}ms to connect!");

            //watch.Restart();
            Console.WriteLine("Joint home 1: " + robot.joint_home(1));
            Console.WriteLine("Joint home 2: " + robot.joint_home(2));
            Console.WriteLine("Joint home 3: " + robot.joint_home(3));
            Console.WriteLine("Joint home 4: " + robot.joint_home(4));

            //Console.ReadKey();
            //Console.WriteLine($"Time to Home: {watch.ElapsedMilliseconds}");
            //Console.ReadKey();

            //while (true)
            //{
            //    if (watch.ElapsedMilliseconds > 15000) break;
            //    robot.get_scara_param();
            //    Console.WriteLine(
            //        $"target: {robot.is_robot_goto_target()}, " +
            //        $"move flag: {robot.move_flag}, " +
            //        $"Angle 1: {robot.angle1}, " +
            //        $"Angle 1 Judge: {robot.angle1_after_judge}, " +
            //        $"Angle 2: {robot.angle2}, " +
            //        $"Angle 2 Judge: {robot.angle2_after_judge}, " +
            //        $"Init Finished: {robot.initial_finish}");
            //    Thread.Sleep(100);
            //}

            //Thread.Sleep(3000);
            //Console.WriteLine(robot.joint_home(3));
            Console.WriteLine("Joint home 3: " + robot.joint_home(3));

            // robot.set_arm_length(200, 200); DO NOT SET ARM LENGTH WHEN JOINT_HOME IS RUNNING!!!
            //Console.WriteLine(robot.change_attitude(20));
            Console.WriteLine("Initialize: " + robot.initial(1, 310));
            //robot.get_scara_param();
            //Console.WriteLine(robot.initial_finish);

            //Console.WriteLine("Unlock Position: " + robot.unlock_position());


            watch.Restart();
            while (true)
            {
                if (watch.ElapsedMilliseconds > 15000) break;
                robot.get_scara_param();
                Console.WriteLine(
                    $"target: {robot.is_robot_goto_target()}, " +
                    $"move flag: {robot.move_flag}, " +
                    $"Angle 1: {robot.angle1}, " +
                    $"Angle 1 Judge: {robot.angle1_after_judge}, " +
                    $"Angle 2: {robot.angle2}, " +
                    $"Angle 2 Judge: {robot.angle2_after_judge}, " +
                    $"Angle 3: {robot.rotation}, " +
                    $"Init Finished: {robot.initial_finish}");
                Thread.Sleep(100);
            }

            robot.set_catch_or_release_accuracy(0.5f);

            //Console.WriteLine("At Home");

            //Console.WriteLine("Going Home");
            //while (robot.move_flag)
            //{
            //    Console.Write(".");
            //    Thread.Sleep(1);
            //}

            //robot.set_arm_length(200, 200);
            //robot.set_catch_or_release_accuracy(0.5f);
            //Console.WriteLine("Unlock Position: " + robot.unlock_position());

            //Console.WriteLine("xyz move: " + robot.xyz_move(1, -10, 50));

            // Console.WriteLine("Cooperation State: " + robot.set_cooperation_fun_state(true));
            //Console.WriteLine("Drag Teach: " + robot.set_drag_teach(true));
        }
    }
}
