using EventArgsLibrary;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;

namespace Positioning2WheelsNS
{
    public class Positioning2Wheels
    {
        int robotID = 10;
        double Tech_Sec = 1 / 50.0; //ou 50f por float
        Location posRobotRefTerrain;
 

        public Positioning2Wheels(int id)
        {
            robotID = id;
            posRobotRefTerrain = new Location(0, 0, 0, 0, 0, 0);
        }
        public void OnOdometryRobotSpeedReceived(object sender, PolarSpeedArgs e)
        {
            // TODO : Eventuellement faire avec e.theta et non e.Vtheta
            posRobotRefTerrain.Vx = e.Vx * Math.Cos(e.Vtheta);
            posRobotRefTerrain.Vy = e.Vx * Math.Sin(e.Vtheta);
            posRobotRefTerrain.Vtheta = e.Vtheta;


            posRobotRefTerrain.X += posRobotRefTerrain.Vx * Tech_Sec;
            posRobotRefTerrain.Y += posRobotRefTerrain.Vy * Tech_Sec;
            posRobotRefTerrain.Theta += posRobotRefTerrain.Vtheta * Tech_Sec;

            OnCalculatedLocation(robotID, posRobotRefTerrain);
        }

        //Output events
        public event EventHandler<LocationArgs> OnCalculatedLocationEvent;
        public virtual void OnCalculatedLocation(int id, Location locationRefTerrain)
        {
            var handler = OnCalculatedLocationEvent;
            if (handler != null)
            {
                handler(this, new LocationArgs { RobotId = id, Location = locationRefTerrain });
            }
        }
    }
}
