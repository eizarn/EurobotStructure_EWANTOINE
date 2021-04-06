using Constants;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Utilities;

namespace StrategyManagerProjetEtudiantNS
{
    public enum TaskDemoMoveState
    {
        GotoPt1,
        GotoPt1EnCours,
        GotoPt2,
        GotoPt2EnCours,
        GotoPt3,
        GotoPt3EnCours,
        GotoPt4,
        GotoPt4EnCours,
    }

    public class TaskDemoMove
    {
        /// <summary>
        /// Une tache est un processus permettant de gérer une partie de code de manière autonome et asynchrone.
        /// On la balaie périodiquement (mais souvent) à une vitesse définie par le Thread.Sleep(ms) en sortie du switch case
        /// 
        /// Afin d'assurer qu'une tache ne sera jamais bloquée, il est FORMELLEMENT INTERDIT d'y mettre des while bloquants
        /// 
        /// Pour éviter de flooder à chaque appel (par exemple si on veut lancer un ordre sur une liaison série),
        ///     on passe l'ordre dans une étape ETAPE_NAME et on passe juste après dans une étape ETAPE_NAME_ATTENTE
        ///     
        /// Pour implanter une temporisation, on a donc recours à une stopwatch initialisée (restart) dans ETAPE_NAME
        ///     et lue dans ETAPE_NAME_ENCOURS
        /// 
        /// La tâche a une référence vers le StrategyXXXX parent qui est passée au constructeur de manière à simplifier
        ///     les accès aux propriétés de ce StrategyXXXX parent sans passer par des events 
        ///     C'est une dérogation à la règle en vigueur dans l'ensemble du projet mais on l'accepte ici vu 
        ///     le nombre d'appel potentiels
        /// 
        /// TaskThread.IsBackground = true; signifie que le Thread est détruit quand on ferme l'application de quelque manière
        ///     que ce soit. Il faut le laisser impérativement.
        ///     
        /// </summary>

        StrategyEurobot parent;
        Thread TaskThread;
        public TaskDemoMoveState state = TaskDemoMoveState.GotoPt1;

        Stopwatch sw = new Stopwatch();
        
        public TaskDemoMove(StrategyEurobot parent)
        {
            this.parent = parent;
            TaskThread = new Thread(TaskThreadProcess);
            TaskThread.IsBackground = true;
            TaskThread.Start();
            sw.Stop();
            sw.Reset();
        }

        public void SetTaskState(TaskDemoMoveState state)
        {
            this.state = state;
        }
             

        void TaskThreadProcess()
        {
            while (true)
            {
                switch (state)
                {
                    case TaskDemoMoveState.GotoPt1:
                        sw.Restart();
                        //Action à effectuer : compléter
                        break;
                    case TaskDemoMoveState.GotoPt1EnCours:
                        if (sw.ElapsedMilliseconds > 8000)
                        {
                            state = TaskDemoMoveState.GotoPt2;
                        }
                        break;
                    case TaskDemoMoveState.GotoPt2:
                        sw.Restart();
                        //Action à effectuer : compléter
                        state = TaskDemoMoveState.GotoPt2EnCours;
                        break;
                    case TaskDemoMoveState.GotoPt2EnCours:
                        if (sw.ElapsedMilliseconds>8000)
                        {
                            state = TaskDemoMoveState.GotoPt3;
                        }                            
                        break;
                    case TaskDemoMoveState.GotoPt3:
                        sw.Restart();
                        //Action à effectuer : compléter
                        state = TaskDemoMoveState.GotoPt3EnCours;
                        break;
                    case TaskDemoMoveState.GotoPt3EnCours:
                        if (sw.ElapsedMilliseconds > 8000)
                            state = TaskDemoMoveState.GotoPt4;
                        break;
                    case TaskDemoMoveState.GotoPt4:
                        sw.Restart();
                        //Action à effectuer : compléter
                        state = TaskDemoMoveState.GotoPt4EnCours;
                        break;
                    case TaskDemoMoveState.GotoPt4EnCours:
                        if (sw.ElapsedMilliseconds > 8000)
                            state = TaskDemoMoveState.GotoPt1;
                        break;
                    default:
                        break;
                }
                Thread.Sleep(20);
            }
        }
    }
}
