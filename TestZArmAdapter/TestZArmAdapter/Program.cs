using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ZArmUnityAdapter;

namespace TestZArmAdapter
{
    class Program
    {
        static void Main(string[] args)
        {
            ZArmAdapter arm = new ZArmAdapter();
            Console.WriteLine("Make new");
            Console.ReadKey();

            arm.InitializeTcpServer();

            Console.WriteLine("Done");
            Console.ReadKey();

        }
    }
}
