using System;
using System.Runtime.InteropServices;

namespace ScaraTesting
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello World!");
        }
    }

    public class ScaraWrapper
    {
        [DllImport("ScaraTesting.dll", EntryPoint = "HelloWorld")]
        public static extern void HelloWorld();
    }
}
