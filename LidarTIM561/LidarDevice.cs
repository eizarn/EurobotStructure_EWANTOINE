using EventArgsLibrary;
using System;
using System.Collections.Generic;
using Utilities;

namespace LidarSickNS
{
    /// <summary>Point décrit par un Lidar.</summary>
    public class LidarPoint
    {
        public override string ToString()
            => $"D: {Distance} ; A: {Angle} ; RSSI: {RSSI}";

        /// <summary>Distance du Lidar au point.</summary>
        public double Distance { get; }

        /// <summary>Angle entre le point et la direction en face du Lidar.</summary>
        public double Angle { get; }

        /// <summary>Valeur RSSI au point.</summary>
        public double RSSI { get; }

        /// <summary>Crée une instance de <see cref="LidarPoint"/>.</summary>
        public LidarPoint(double distance, double angle, double rssi = 0)
        {
            Distance = distance;
            Angle = angle;
            RSSI = rssi;
        }
    }

    /// <summary>
    /// Décrit un capteur Lidar donnant une liste de points et une potentielle liste de RSSI.
    /// </summary>
    public abstract class LidarDevice
    {
        #region Properties

        /// <summary>Angle minimum de scan.</summary>
        public double AngleMin { get; set; } = 0;

        /// <summary>Angle maximum de scan.</summary>
        public double AngleMax { get; set; } = 360;

        /// <summary>Position du Lidar dans le référentiel robot.</summary>
        public Location Location { get; set; } = new Location(0,0,0,0,0,0);

        /// <summary>Indique si le Lidar est retourné.</summary>
        public bool IsUpsideDown { get; set; } = false;

        /// <summary>Liste des points vus par le Lidar en coordonnées polaires (avec RSSI).</summary>
        public List<LidarPoint> LidarPoints { get; } = new List<LidarPoint>();
        public List<PolarPointRssi> LidarPolarPointsList { get; } = new List<PolarPointRssi>();

        #endregion
        #region Events & Methods

        /// <summary>Résolu lorsque des points sont reçus.</summary>
        public event LidarPointsReadyEventHandler PointsAvailable;

        /// <summary>À appeler pour lancer l'event <see cref="PointsAvailable"/>.</summary>
        protected void OnLidarPointsReady()
            => PointsAvailable?.Invoke(this, new LidarPointsReadyEventArgs(LidarPoints));

        /// <summary>Démarre le Lidar.</summary>
        public abstract void Start();

        /// <summary>Arrête le Lidar.</summary>
        public abstract void Stop();

        public event EventHandler<RawLidarArgs> OnLidarDecodedFrameEvent;
        protected void OnLidarDecodedFrame(int id, List<PolarPointRssi> ptList, int lidarFrameNumber = 0)
        {
            var handler = OnLidarDecodedFrameEvent;
            if (handler != null)
            {
                handler(this, new RawLidarArgs { RobotId = id, PtList = ptList, LidarFrameNumber = lidarFrameNumber });
            }
        }

        #endregion
    }

    //public class RawLidarArgs : EventArgs
    //{
    //    public int RobotId { get; set; }
    //    public List<PolarPointRssi> PtList { get; set; }
    //    public int LidarFrameNumber { get; set; }
    //}

    //public class PolarPointRssi
    //{
    //    public virtual double Distance { get; set; }
    //    public virtual double Angle { get; set; }
    //    public virtual double Rssi { get; set; }

    //    public PolarPointRssi(double angle, double distance, double rssi)
    //    {
    //        Distance = distance;
    //        Angle = angle;
    //        Rssi = rssi;
    //    }
    //    public PolarPointRssi()
    //    {

    //    }
    //}

    /// <summary>EventHandler de points Lidar traités.</summary>
    public delegate void LidarPointsReadyEventHandler(object sender, LidarPointsReadyEventArgs e);

    /// <summary>Arguments contenant une liste de points trouvés dans le référentiel du robot.</summary>
    public class LidarPointsReadyEventArgs : EventArgs
    {
        /// <summary>Points dans le référentiel Lidar en coordonnées polaires.</summary>
        public readonly List<LidarPoint> LidarPoints;

        /// <summary>Crée une instance de <see cref="LidarPointsReadyEventArgs"/>.</summary>
        public LidarPointsReadyEventArgs(List<LidarPoint> lidarPoints)
        {
            LidarPoints = lidarPoints;
        }
    }
}
