using EventArgsLibrary;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;

namespace TrajectoryGeneratorNonHolonomeNS
{
    public class TrajectoryGeneratorNonHolonome
    {

        
        int robotId = 10;
        double Tech_Sec = 1 / 50.0; //ou 50f por float
      
        Location currentLocationRefTerrain;
        Location ghostLocationRefTerrain;
        
        PointD destination = new PointD(1, 1);
        PointD pointCible = new PointD(0,0);

        double accelLineaire, accelAngulaire;
        double vitesseLineaireMax, vitesseAngulaireMax;

        AsservissementPID PID_Position_Lineaire;
        AsservissementPID PID_Position_Angulaire;

        //trucs du ghost state machine
        public enum GhostState { idle, rotation, avance };
        GhostState ghostState = GhostState.idle;

        public TrajectoryGeneratorNonHolonome(int id)
        {
            robotId = id;
            InitRobotPosition(0, 0, 0);
            InitPositionPID();

            //Initialisation des vitesse et accélérations souhaitées
            accelLineaire = 0.02; //en m.s-2
            accelAngulaire = 0.1 * Math.PI * 1.0; //en rad.s-2 //TODO l'axel richter angulaire = 0.05 

            vitesseLineaireMax = 1; //en m.s-1               
            vitesseAngulaireMax = 1 * Math.PI * 1.0; //en rad.s-1
        }

        void InitPositionPID()
        {
            PID_Position_Lineaire = new AsservissementPID(20.0, 10.0, 0, 100, 100, 1);
            PID_Position_Angulaire = new AsservissementPID(20.0, 10.0, 0, 5 * Math.PI, 5 * Math.PI, Math.PI);
        }

        public void InitRobotPosition(double x, double y, double theta)
        {
            Location old_currectLocation = currentLocationRefTerrain;
            currentLocationRefTerrain = new Location(x, y, theta, 0, 0, 0);
            //wayPointLocation = new Location(100, 100, theta, 0, 0, 0);
            ghostLocationRefTerrain = new Location(x, y, theta, 0, 0, 0);
            PIDPositionReset();
        }

        public void SetPosition(object sender, PositionArgs e)
        {
            ghostState = GhostState.rotation;
            destination = new PointD(e.X, e.Y);
        }

        public void OnPhysicalPositionReceived(object sender, LocationArgs e)
        {

            if (robotId == e.RobotId)
            {

                currentLocationRefTerrain = e.Location;
                CalculateGhostPosition();
                PIDPosition();

            }
        }


        void CalculateGhostPosition()
        {
            double vitesseLinearGhost = Math.Sqrt(Math.Pow(ghostLocationRefTerrain.Vy, 2) + Math.Pow(ghostLocationRefTerrain.Vx, 2));

            switch (ghostState)
            {
                case GhostState.idle:
                    Console.WriteLine("Idle");

                    ghostLocationRefTerrain.Vx = 0;
                    ghostLocationRefTerrain.Vy = 0;
                    ghostLocationRefTerrain.Vtheta = 0;

                   double destination_Theta = Math.Atan2(destination.Y - currentLocationRefTerrain.Y,
                                                       destination.X - currentLocationRefTerrain.X);

                    if (ghostLocationRefTerrain.Theta != destination_Theta) ghostState = GhostState.rotation;

                    break;

                case GhostState.rotation:


                    double ThetaCorrect = Math.Atan2(destination.Y - ghostLocationRefTerrain.Y, destination.X - ghostLocationRefTerrain.X);
                    double ThetaStopDistance = Math.Pow(ghostLocationRefTerrain.Vtheta, 2) / (2 * accelAngulaire);
                    double ThetaRestant = ThetaCorrect - Toolbox.Modulo2PiAngleRad(ghostLocationRefTerrain.Theta);

                    if(ThetaStopDistance < ThetaRestant) 
                        ghostLocationRefTerrain.Vtheta += accelAngulaire * Tech_Sec;
                    if(ThetaStopDistance >= ThetaRestant)
                        ghostLocationRefTerrain.Vtheta -= accelAngulaire * Tech_Sec;
                    if(ThetaRestant <= 0.001)
                        ghostState = GhostState.avance;

                    break;

                case GhostState.avance:
                    ghostLocationRefTerrain.Vtheta = 0;
                    //On injecte la position du ghost dans positionGhost
                    PointD positionGhost = new PointD(ghostLocationRefTerrain.X, ghostLocationRefTerrain.Y);

                    //On calcule un point sur la droite (faut demander pourquoi parce que je ne sais pas)
                    PointD pointDroite = new PointD(ghostLocationRefTerrain.X + Math.Cos(ghostLocationRefTerrain.Theta),
                                                    ghostLocationRefTerrain.Y + Math.Cos(ghostLocationRefTerrain.Theta));

                    
                    //On détermine le point cible Projeté
                    pointCible = Toolbox.ProjectionPointToLine(destination, positionGhost, pointDroite);

                    //On Calcule la distance d'arret du ghost
                    double DistanceStop = Math.Pow(vitesseLinearGhost, 2) / (2 * accelLineaire);

                    //On Calcule la distance cible 
                    double distanceRestante = Math.Sqrt(Math.Pow(pointCible.Y - ghostLocationRefTerrain.Y, 2) + Math.Pow(pointCible.X - ghostLocationRefTerrain.X, 2));

                    //Calcule des angles
                    double angleCible = Math.Atan2(destination.Y - ghostLocationRefTerrain.Y, destination.X - ghostLocationRefTerrain.X);
                    double angleRobotCible = Toolbox.ModuloByAngle(ghostLocationRefTerrain.Theta, angleCible) - ghostLocationRefTerrain.Theta;


                    if(DistanceStop < Math.Abs(distanceRestante))
                    {
                        if(Math.Abs(vitesseLinearGhost) < vitesseLineaireMax)
                        {
                            if (Math.Abs(angleRobotCible) < Math.PI / 2)
                                vitesseLinearGhost += accelLineaire * Tech_Sec;
                            else
                                vitesseLinearGhost -= accelLineaire * Tech_Sec;
                        }
                    }
                    else
                    {
                    //Freins shifter pro
                        if(Math.Abs(vitesseLinearGhost) < Math.PI/2)
                                vitesseLinearGhost -= accelLineaire * Tech_Sec;
                        else
                                vitesseLinearGhost += accelLineaire * Tech_Sec;
                    }

                    ghostLocationRefTerrain.X += (vitesseLinearGhost * 1/Tech_Sec) * Math.Cos(ghostLocationRefTerrain.Theta);
                    ghostLocationRefTerrain.Y += (vitesseLinearGhost * 1/Tech_Sec) * Math.Sin(ghostLocationRefTerrain.Theta);
                    if (distanceRestante < 0.002)
                        ghostState = GhostState.idle;
                    break;
            }


            ghostLocationRefTerrain.Theta += ghostLocationRefTerrain.Vtheta * Tech_Sec;


            //On renvoie la position du ghost pour affichage
            OnGhostLocation(robotId, ghostLocationRefTerrain);
        }

        void PIDPosition()
        {
            //A remplir
            double vLineaireRobot=0, vAngulaireRobot=0;


            //Si tout c'est bien passé, on envoie les vitesses consigne.
            OnSpeedConsigneToRobot(robotId, (float)vLineaireRobot, (float)vAngulaireRobot);
        }

        void PIDPositionReset()
        {
            if (PID_Position_Angulaire != null && PID_Position_Lineaire != null)
            {
                PID_Position_Lineaire.ResetPID(0);
                PID_Position_Angulaire.ResetPID(0);
            }
        }

        /*************************************** Outgoing Events ************************************/

        public event EventHandler<LocationArgs> OnGhostLocationEvent;
        public virtual void OnGhostLocation(int id, Location loc)
        {
            var handler = OnGhostLocationEvent;
            if (handler != null)
            {
                handler(this, new LocationArgs { RobotId = id, Location = loc });
            }
        }

        public event EventHandler<PolarSpeedArgs> OnSpeedConsigneEvent;
        public virtual void OnSpeedConsigneToRobot(int id, float vLineaire, float vAngulaire)
        {
            var handler = OnSpeedConsigneEvent;
            if (handler != null)
            {
                handler(this, new PolarSpeedArgs { RobotId = id, Vx = vLineaire, Vy = 0, Vtheta = vAngulaire});
            }
        }
    }
}
