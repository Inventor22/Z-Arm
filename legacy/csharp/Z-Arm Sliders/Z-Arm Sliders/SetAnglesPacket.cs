using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets
{
    [Serializable]
    public class SetAnglesPacket
    {
        public float ShoulderAngle { get; set; }
        public float ElbowAngle { get; set; }
        public float WristAngle { get; set; }
        public float Height { get; set; }
        public float Speed { get; set; }
    }
}
