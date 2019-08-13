using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using TcpserverExDll;
using ControlBeanExDll;
using System.Threading;

namespace Z_Arm_Class
{
    class Program
    {

        static void Main(string[] args)
        {
            Console.WriteLine("Hello ZArm!");

            ZArm robot = new ZArm(59);

            try
            {
                robot.Initialize().Wait();

                robot.AngularVelocity = 200;
                robot.GoHome();                
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }

            Console.WriteLine("At Home");
            Console.ReadKey();
        }
    }
}
