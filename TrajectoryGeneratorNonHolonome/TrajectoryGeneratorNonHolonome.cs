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
        Location wayPointLocation = new Location();


        int robotId = 10;
        double Tech_Sec = 1 / 50.0; //ou 50f por float

        Location currentLocationRefTerrain;
        Location ghostLocationRefTerrain;

        PointD destination = new PointD(10, 1);
        PointD pointCible = new PointD(0, 0);

        double accelLineaire, accelAngulaire;
        double accelLineaireMax, accelAngulaireMax;
        double vitesseLineaireMax, vitesseAngulaireMax;
        //double vitesseLinearGhost = Math.Sqrt(Math.Pow(ghostLocationRefTerrain.Vy, 2) + Math.Pow(ghostLocationRefTerrain.Vx, 2));
        double vitesseLinearGhost = 0;
        int direction;

        double toleranceAngulaire = 0.01, toleranceLineaire = 0.1;

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
            accelLineaireMax = 3; //0.5; //en m.s-2
            accelAngulaireMax = 2/*0.2*/ * Math.PI * 1.0; //en rad.s-2 //TODO l'axel richter angulaire = 0.

            vitesseLineaireMax = 1; //1 //en m.s-1               
            vitesseAngulaireMax = 1 * Math.PI * 1.0; //en rad.s-1
        }

        void InitPositionPID()
        {
            PID_Position_Lineaire = new AsservissementPID(0, 0, 0, 100, 100, 1);
            PID_Position_Angulaire = new AsservissementPID(0, 0, 1, 5 * Math.PI, 5 * Math.PI, Math.PI);
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

        public void OnWaypointReceived(object sender, PositionArgs destination)
        {
            /// Mise à jour du waypoint courant
            pointCible.X = destination.X;
            pointCible.Y = destination.Y;
            ghostState = GhostState.rotation;
        }


        void CalculateGhostPosition()
        {
            Console.WriteLine("x={0}, y={1}", wayPointLocation.X, wayPointLocation.Y);

            switch (ghostState)
            {
                case GhostState.idle:
                    Console.WriteLine("Idle");

                    ghostLocationRefTerrain.Vx = 0;
                    ghostLocationRefTerrain.Vy = 0;
                    ghostLocationRefTerrain.Vtheta = 0;

                    accelAngulaire = accelAngulaireMax;
                    accelLineaire = accelLineaireMax;

                    double destination_Theta = Math.Atan2(destination.Y - currentLocationRefTerrain.Y,
                                                          destination.X - currentLocationRefTerrain.X);

                    if (Math.Abs(destination_Theta - ghostLocationRefTerrain.Theta) < toleranceAngulaire)
                    {
                        ghostState = GhostState.rotation;

                        //if (destination_Theta > ghostLocationRefTerrain.Theta)
                        //    direction = 1;
                        //else
                        //    direction = -1;
                    }

                    break;

                case GhostState.rotation:

                    double ThetaDestination = Math.Atan2(pointCible.Y - ghostLocationRefTerrain.Y, pointCible.X - ghostLocationRefTerrain.X);
                    double ThetaRestant = ThetaDestination - Toolbox.Modulo2PiAngleRad(ghostLocationRefTerrain.Theta);
                    double ThetaStopDistance = Math.Pow(ghostLocationRefTerrain.Vtheta, 2) / (2 * accelAngulaire);

                    if (ThetaRestant > 0)
                    {
                        if (ghostLocationRefTerrain.Vtheta < 0) // anormal
                            ghostLocationRefTerrain.Vtheta -= accelAngulaire * Tech_Sec;
                        else
                        {
                            if (ThetaRestant > ThetaStopDistance)
                            {
                                if (ghostLocationRefTerrain.Vtheta < vitesseAngulaireMax) // On a pas encore atteint le Vmax donc on accélère
                                    ghostLocationRefTerrain.Vtheta += accelAngulaire * Tech_Sec;
                            }
                            else
                                ghostLocationRefTerrain.Vtheta -= accelAngulaire * Tech_Sec;
                        }
                    }
                    else // l'angle restant à parcourir est inférieur à 0
                    {
                        if (ghostLocationRefTerrain.Vtheta > 0) // anormal
                            ghostLocationRefTerrain.Vtheta += accelAngulaire * Tech_Sec;
                        else
                        {
                            if (Math.Abs(ThetaRestant) > ThetaStopDistance)
                            {
                                if (ghostLocationRefTerrain.Vtheta > -vitesseAngulaireMax) // On a pas encore atteint le Vmax donc on accélère en négatif
                                    ghostLocationRefTerrain.Vtheta -= accelAngulaire * Tech_Sec;
                            }
                            else
                                ghostLocationRefTerrain.Vtheta += accelAngulaire * Tech_Sec;
                        }
                    }
                    if (Math.Abs(ThetaRestant) < toleranceAngulaire)
                        ghostState = GhostState.avance;

                    //On actualise l'angle du ghost
                    ghostLocationRefTerrain.Theta += ghostLocationRefTerrain.Vtheta * Tech_Sec;

                    break;

                case GhostState.avance:

                    ghostLocationRefTerrain.Vtheta = 0;
                    //On injecte la position du ghost dans positionGhost
                    PointD positionGhost = new PointD(ghostLocationRefTerrain.X, ghostLocationRefTerrain.Y);

                    //On calcule un point sur la droite
                    PointD pointDroite = new PointD(ghostLocationRefTerrain.X + Math.Cos(ghostLocationRefTerrain.Theta),
                                                    ghostLocationRefTerrain.Y + Math.Sin(ghostLocationRefTerrain.Theta));


                    //On Calcule la distance d'arret du ghost
                    double DistanceStop = Math.Pow(vitesseLinearGhost, 2) / (2 * accelLineaire);

                    //On Calcule la distance cible 
                    double distanceRestante = Math.Sqrt(Math.Pow(destination.Y - ghostLocationRefTerrain.Y, 2) + Math.Pow(destination.X - ghostLocationRefTerrain.X, 2));


                    //Calcul des angles
                    double angleCible = Math.Atan2(destination.Y - ghostLocationRefTerrain.Y, destination.X - ghostLocationRefTerrain.X);
                    double angleRobotCible = angleCible - Toolbox.ModuloByAngle(angleCible, ghostLocationRefTerrain.Theta);

                    //On va regarder si la cible est devant ou derrière nous
                    bool cibleDevant = true;
                    if (Math.Abs(angleRobotCible) > Math.PI / 2)
                        cibleDevant = false;

                    double CoeffMaj = 2;

                    if (cibleDevant)
                    {
                        Console.WriteLine("cible Devant");
                        //On est dans le cas où la cible est devant vous, donc notre vitesse doit absolument être POSITIVE !
                        //Donc on se questionne : " EST CE QUE NOTRE VITESSE EST BIEN POSITIVE ?
                        if (vitesseLinearGhost < 0)
                        {
                            //On tombe dans le cas où ce n'est pas positive donc on inverse le sens de la vitesse
                            vitesseLinearGhost += accelLineaire * Tech_Sec;
                        }
                        else
                        {
                            if (distanceRestante > DistanceStop)
                            {
                                //On regarde si on est à Vmax 
                                if (Math.Abs(vitesseLinearGhost) < vitesseLineaireMax)
                                    vitesseLinearGhost += accelLineaire * Tech_Sec;
                            }
                            else
                                vitesseLinearGhost -= accelLineaire * Tech_Sec;
                        }

                    }
                    else
                    {
                        Console.WriteLine("cible Derrière");
                        //On est dans le cas où la cible est derrière nous, donc notre vitesse doit absolument être NÉGATIVE ! 
                        //Donc on se questionne : "EST CE QUE LA VITESSE EST BIEN NÉGATIVE ?" 

                        if (vitesseLinearGhost > 0)
                        {
                            //On inverse le sens de l'acceleration pour revenir à une vitesse négative
                            vitesseLinearGhost -= accelLineaire * Tech_Sec;
                        }
                        else
                        {
                            if (distanceRestante > DistanceStop)
                            {
                                //On regarde si on est à Vmax
                                if (Math.Abs(vitesseLinearGhost) < vitesseLineaireMax)
                                    vitesseLinearGhost -= accelLineaire * Tech_Sec;
                            }
                            else
                                vitesseLinearGhost += accelLineaire * Tech_Sec;
                        }

                    }

                    if (distanceRestante < toleranceLineaire)
                    {
                        ghostState = GhostState.idle;
                        Console.WriteLine("Erreur: {0}", distanceRestante);
                    }

                    ghostLocationRefTerrain.X += (vitesseLinearGhost * Tech_Sec) * Math.Cos(ghostLocationRefTerrain.Theta);
                    ghostLocationRefTerrain.Y += (vitesseLinearGhost * Tech_Sec) * Math.Sin(ghostLocationRefTerrain.Theta);

                    Console.WriteLine(vitesseLinearGhost);


                    break;
            }

            //On renvoie la position du ghost pour affichage
            OnGhostLocation(robotId, ghostLocationRefTerrain);
        }

        void PIDPosition()
        {
            //Calcul de l'erreur Angulaire
            double erreur_Angulaire = ghostLocationRefTerrain.Theta - currentLocationRefTerrain.Theta;
            double vAngulaireRobot = PID_Position_Angulaire.CalculatePDoutput(erreur_Angulaire, Tech_Sec);

            //Calcul de l'erreur linéaire ghostLocationRefTerrain / currentLocationRefTerrain
            PointD GhostPosition = new PointD(ghostLocationRefTerrain.X, ghostLocationRefTerrain.Y);
            PointD pointSegRobot2 = new PointD(currentLocationRefTerrain.X + Math.Cos(currentLocationRefTerrain.Theta), currentLocationRefTerrain.Y + Math.Sin(currentLocationRefTerrain.Theta));
            PointD pointSegRobot1 = new PointD(currentLocationRefTerrain.X, currentLocationRefTerrain.Y);
            PointD Projection_ghost = Toolbox.ProjectionPointToLine(GhostPosition, pointSegRobot1, pointSegRobot2);

            double thetaCible = Math.Atan2(ghostLocationRefTerrain.Y - currentLocationRefTerrain.Y, ghostLocationRefTerrain.X - currentLocationRefTerrain.X);
            double ecratCapCibleRobot = thetaCible - Toolbox.ModuloByAngle(thetaCible, currentLocationRefTerrain.Theta);

            int param = 1;
            if (Math.Abs(ecratCapCibleRobot) > Math.PI / 2)
                param = -1;

            double ErreurLin = Toolbox.Distance(Projection_ghost, new PointD(currentLocationRefTerrain.X, currentLocationRefTerrain.Y)) * param;
            Console.WriteLine("ErreurLin: {0}", ErreurLin);

            double vLineaireRobot = PID_Position_Lineaire.CalculatePDoutput(ErreurLin, Tech_Sec);

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
                handler(this, new PolarSpeedArgs { RobotId = id, Vx = vLineaire, Vy = 0, Vtheta = vAngulaire });
            }
        }
    }
}






//using EventArgsLibrary;
//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.Threading.Tasks;
//using Utilities;

//namespace TrajectoryGeneratorNonHolonomeNS
//{
//    public class TrajectoryGeneratorNonHolonome
//    {
//        Location wayPointLocation = new Location();


//        int robotId = 10;
//        double Tech_Sec = 1 / 50.0; //ou 50f por float

//        Location currentLocationRefTerrain;
//        Location ghostLocationRefTerrain;

//        PointD destination = new PointD(1, 1);
//        PointD pointCible = new PointD(0,0);

//        double accelLineaire, accelAngulaire;
//        double accelLineaireMax, accelAngulaireMax;
//        double vitesseLineaireMax, vitesseAngulaireMax;
//        //double vitesseLinearGhost = Math.Sqrt(Math.Pow(ghostLocationRefTerrain.Vy, 2) + Math.Pow(ghostLocationRefTerrain.Vx, 2));
//        double vitesseLinearGhost = 0;
//        int direction;

//        double toleranceAngulaire = 0.01, toleranceLineaire = 0.1;

//        AsservissementPID PID_Position_Lineaire;
//        AsservissementPID PID_Position_Angulaire;

//        //trucs du ghost state machine
//        public enum GhostState { idle, rotation, avance };
//        GhostState ghostState = GhostState.idle;

//        public TrajectoryGeneratorNonHolonome(int id)
//        {
//            robotId = id;
//            InitRobotPosition(0, 0, 0);
//            InitPositionPID();

//            //Initialisation des vitesse et accélérations souhaitées
//            accelLineaireMax = 3; //0.5; //en m.s-2
//            accelAngulaireMax = 2/*0.2*/ * Math.PI * 1.0; //en rad.s-2 //TODO l'axel richter angulaire = 0.

//            vitesseLineaireMax = 1; //1 //en m.s-1               
//            vitesseAngulaireMax = 1 * Math.PI * 1.0; //en rad.s-1
//        }

//        void InitPositionPID()
//        {
//            PID_Position_Lineaire = new AsservissementPID(0, 0, 0, 100, 100, 1);
//            PID_Position_Angulaire = new AsservissementPID(0, 0, 1, 5 * Math.PI, 5 * Math.PI, Math.PI);
//        }

//        public void InitRobotPosition(double x, double y, double theta)
//        {
//            Location old_currectLocation = currentLocationRefTerrain;
//            currentLocationRefTerrain = new Location(x, y, theta, 0, 0, 0);
//            //wayPointLocation = new Location(100, 100, theta, 0, 0, 0);
//            ghostLocationRefTerrain = new Location(x, y, theta, 0, 0, 0);
//            PIDPositionReset();
//        }

//        public void SetPosition(object sender, PositionArgs e)
//        {
//            ghostState = GhostState.rotation;
//            destination = new PointD(e.X, e.Y);
//        }

//        public void OnPhysicalPositionReceived(object sender, LocationArgs e)
//        {

//            if (robotId == e.RobotId)
//            {

//                currentLocationRefTerrain = e.Location;
//                CalculateGhostPosition();
//                PIDPosition();
//            }
//        }

//        public void OnWaypointReceived(object sender, PositionArgs destination)
//        {
//            /// Mise à jour du waypoint courant
//            pointCible.X = destination.X;
//            pointCible.Y = destination.Y;
//            ghostState = GhostState.rotation;
//        }


//        void CalculateGhostPosition()
//        {
//            Console.WriteLine("x={0}, y={1}", wayPointLocation.X, wayPointLocation.Y);

//            switch (ghostState)
//            {
//                case GhostState.idle:
//                    Console.WriteLine("Idle");

//                    ghostLocationRefTerrain.Vx = 0;
//                    ghostLocationRefTerrain.Vy = 0;
//                    ghostLocationRefTerrain.Vtheta = 0;

//                    accelAngulaire = accelAngulaireMax;
//                    accelLineaire = accelLineaireMax;

//                    double destination_Theta = Math.Atan2(destination.Y - currentLocationRefTerrain.Y,
//                                                          destination.X - currentLocationRefTerrain.X);

//                    if (Math.Abs(destination_Theta - ghostLocationRefTerrain.Theta) < toleranceAngulaire)
//                    {
//                        ghostState = GhostState.rotation;

//                        //if (destination_Theta > ghostLocationRefTerrain.Theta)
//                        //    direction = 1;
//                        //else
//                        //    direction = -1;
//                    }

//                    break;

//                case GhostState.rotation:

//                    double ThetaDestination = Math.Atan2(pointCible.Y - ghostLocationRefTerrain.Y, pointCible.X - ghostLocationRefTerrain.X);
//                    double ThetaRestant = ThetaDestination - Toolbox.Modulo2PiAngleRad(ghostLocationRefTerrain.Theta);
//                    double ThetaStopDistance = Math.Pow(ghostLocationRefTerrain.Vtheta, 2) / (2 * accelAngulaire);

//                    if (ThetaRestant > 0)
//                    {
//                        if (ghostLocationRefTerrain.Vtheta < 0) // anormal
//                            ghostLocationRefTerrain.Vtheta -= accelAngulaire * Tech_Sec;
//                        else
//                        {
//                            if (ThetaRestant > ThetaStopDistance)
//                            {
//                                if (ghostLocationRefTerrain.Vtheta < vitesseAngulaireMax) // On a pas encore atteint le Vmax donc on accélère
//                                    ghostLocationRefTerrain.Vtheta += accelAngulaire * Tech_Sec;
//                            }
//                            else
//                                ghostLocationRefTerrain.Vtheta -= accelAngulaire * Tech_Sec;
//                        }
//                    }
//                    else // l'angle restant à parcourir est inférieur à 0
//                    {
//                        if (ghostLocationRefTerrain.Vtheta > 0) // anormal
//                            ghostLocationRefTerrain.Vtheta += accelAngulaire * Tech_Sec;
//                        else
//                        {
//                            if (Math.Abs(ThetaRestant) > ThetaStopDistance)
//                            {
//                                if (ghostLocationRefTerrain.Vtheta > -vitesseAngulaireMax) // On a pas encore atteint le Vmax donc on accélère en négatif
//                                    ghostLocationRefTerrain.Vtheta -= accelAngulaire * Tech_Sec;
//                            }
//                            else
//                                ghostLocationRefTerrain.Vtheta += accelAngulaire * Tech_Sec;
//                        }
//                    }
//                    if (Math.Abs(ThetaRestant) < toleranceAngulaire)
//                        ghostState = GhostState.avance;

//                    //On actualise l'angle du ghost
//                    ghostLocationRefTerrain.Theta += ghostLocationRefTerrain.Vtheta * Tech_Sec;

//                    break;

//                case GhostState.avance:

//                    ghostLocationRefTerrain.Vtheta = 0;
//                    //On injecte la position du ghost dans positionGhost
//                    PointD positionGhost = new PointD(ghostLocationRefTerrain.X, ghostLocationRefTerrain.Y);

//                    //On calcule un point sur la droite
//                    PointD pointDroite = new PointD(ghostLocationRefTerrain.X + Math.Cos(ghostLocationRefTerrain.Theta),
//                                                    ghostLocationRefTerrain.Y + Math.Sin(ghostLocationRefTerrain.Theta));


//                    //On Calcule la distance d'arret du ghost
//                    double DistanceStop = Math.Pow(vitesseLinearGhost, 2) / (2 * accelLineaire);

//                    //On Calcule la distance cible 
//                    double distanceRestante = Math.Sqrt(Math.Pow(destination.Y - ghostLocationRefTerrain.Y, 2) + Math.Pow(destination.X - ghostLocationRefTerrain.X, 2));


//                    //Calcul des angles
//                    double angleCible = Math.Atan2(destination.Y - ghostLocationRefTerrain.Y, destination.X - ghostLocationRefTerrain.X);
//                    double angleRobotCible = angleCible - Toolbox.ModuloByAngle(angleCible, ghostLocationRefTerrain.Theta);

//                    //On va regarder si la cible est devant ou derrière nous
//                    bool cibleDevant = true;
//                    if (Math.Abs(angleRobotCible) > Math.PI / 2)
//                        cibleDevant = false;

//                    double CoeffMaj = 2;

//                    if(cibleDevant)
//                    {
//                        Console.WriteLine("cible Devant");
//                        //On est dans le cas où la cible est devant vous, donc notre vitesse doit absolument être POSITIVE !
//                        //Donc on se questionne : " EST CE QUE NOTRE VITESSE EST BIEN POSITIVE ?
//                        if (vitesseLinearGhost < 0)
//                        {
//                            //On tombe dans le cas où ce n'est pas positive donc on inverse le sens de la vitesse
//                            vitesseLinearGhost += accelLineaire * Tech_Sec;
//                        }     
//                        else
//                        {
//                            if(distanceRestante > DistanceStop)
//                            {
//                                //On regarde si on est à Vmax 
//                                if(Math.Abs(vitesseLinearGhost) < vitesseLineaireMax)
//                                    vitesseLinearGhost += accelLineaire * Tech_Sec;
//                            }
//                            else
//                                vitesseLinearGhost -= accelLineaire * Tech_Sec;
//                        }

//                    }
//                    else
//                    {
//                        Console.WriteLine("cible Derrière");
//                        //On est dans le cas où la cible est derrière nous, donc notre vitesse doit absolument être NÉGATIVE ! 
//                        //Donc on se questionne : "EST CE QUE LA VITESSE EST BIEN NÉGATIVE ?" 

//                        if (vitesseLinearGhost > 0)
//                        {
//                            //On inverse le sens de l'acceleration pour revenir à une vitesse négative
//                            vitesseLinearGhost -= accelLineaire * Tech_Sec;
//                        }
//                        else
//                        {
//                            if(distanceRestante > DistanceStop)
//                            {
//                                //On regarde si on est à Vmax
//                                if (Math.Abs(vitesseLinearGhost) < vitesseLineaireMax)
//                                    vitesseLinearGhost -= accelLineaire * Tech_Sec;
//                            }
//                            else
//                                vitesseLinearGhost += accelLineaire * Tech_Sec;
//                        }

//                    }

//                    if (distanceRestante < toleranceLineaire)
//                    {
//                        ghostState = GhostState.idle;
//                        Console.WriteLine("Erreur: {0}", distanceRestante);
//                    }

//                   ghostLocationRefTerrain.X += (vitesseLinearGhost *  Tech_Sec) * Math.Cos(ghostLocationRefTerrain.Theta);
//                   ghostLocationRefTerrain.Y += (vitesseLinearGhost *  Tech_Sec) * Math.Sin(ghostLocationRefTerrain.Theta);

//                   Console.WriteLine(vitesseLinearGhost);


//                   break;
//            }

//            //On renvoie la position du ghost pour affichage
//            OnGhostLocation(robotId, ghostLocationRefTerrain);
//        }

//        void PIDPosition()
//        {
//            //Calcul de l'erreur Angulaire
//            double erreur_Angulaire = ghostLocationRefTerrain.Theta - currentLocationRefTerrain.Theta;
//            double vAngulaireRobot = PID_Position_Angulaire.CalculatePDoutput(erreur_Angulaire, Tech_Sec);

//            //Calcul de l'erreur linéaire ghostLocationRefTerrain / currentLocationRefTerrain
//            PointD GhostPosition = new PointD(ghostLocationRefTerrain.X, ghostLocationRefTerrain.Y);
//            PointD pointSegRobot2 = new PointD(currentLocationRefTerrain.X + Math.Cos(currentLocationRefTerrain.Theta), currentLocationRefTerrain.Y + Math.Sin(currentLocationRefTerrain.Theta));
//            PointD pointSegRobot1 = new PointD(currentLocationRefTerrain.X, currentLocationRefTerrain.Y);
//            PointD Projection_ghost = Toolbox.ProjectionPointToLine(GhostPosition, pointSegRobot1, pointSegRobot2);

//            double thetaCible = Math.Atan2(ghostLocationRefTerrain.Y - currentLocationRefTerrain.Y, ghostLocationRefTerrain.X - currentLocationRefTerrain.X);
//            double ecratCapCibleRobot = thetaCible - Toolbox.ModuloByAngle(thetaCible, currentLocationRefTerrain.Theta);

//            int param = 1;
//            if (Math.Abs(ecratCapCibleRobot) > Math.PI / 2)
//                param = -1;

//            double ErreurLin = Toolbox.Distance(Projection_ghost, new PointD(currentLocationRefTerrain.X, currentLocationRefTerrain.Y)) * param;
//            Console.WriteLine("ErreurLin: {0}", ErreurLin);

//            double vLineaireRobot = PID_Position_Lineaire.CalculatePDoutput(ErreurLin, Tech_Sec);

//            //Si tout c'est bien passé, on envoie les vitesses consigne.
//            OnSpeedConsigneToRobot(robotId, (float)vLineaireRobot, (float)vAngulaireRobot);
//        }

//        void PIDPositionReset()
//        {
//            if (PID_Position_Angulaire != null && PID_Position_Lineaire != null)
//            {
//                PID_Position_Lineaire.ResetPID(0);
//                PID_Position_Angulaire.ResetPID(0);
//            }
//        }

//        /*************************************** Outgoing Events ************************************/

//        public event EventHandler<LocationArgs> OnGhostLocationEvent;
//        public virtual void OnGhostLocation(int id, Location loc)
//        {
//            var handler = OnGhostLocationEvent;
//            if (handler != null)
//            {
//                handler(this, new LocationArgs { RobotId = id, Location = loc });
//            }
//        }

//        public event EventHandler<PolarSpeedArgs> OnSpeedConsigneEvent;
//        public virtual void OnSpeedConsigneToRobot(int id, float vLineaire, float vAngulaire)
//        {
//            var handler = OnSpeedConsigneEvent;
//            if (handler != null)
//            {
//                handler(this, new PolarSpeedArgs { RobotId = id, Vx = vLineaire, Vy = 0, Vtheta = vAngulaire});
//            }
//        }
//    }
//}
