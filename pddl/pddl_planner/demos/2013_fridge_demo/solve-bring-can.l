#!/usr/bin/env roseus

(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")

;;
;; problem
;;
(setq *problem* (instance pddl-problem :init :name 'bring_can :domain 'fridgedemo))

;;使用する変数の宣言
(send *problem* :objects
      '((START . spot)
        (FRONTFRIDGE . spot)
        (PREGRASP . spot)
        (PRECLOSE . spot)
        (SOMEWHERE . spot)
        (CAN . object)
        ))

;;初期条件
(send *problem* :initial-condition
      '((CLOSED)
        (AT START)
        ))

;;終了条件
(send *problem* :goal-condition
      '((ONHAND CAN)
        (CLOSED)
        (AT START)
        ))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'fridgedemo))
(send *domain* :requirements '(:typing))
(send *domain* :types '(object spot))
(send *domain* :predicates '((ONHAND ?OBJ - object)
                             (AT ?SPOT - spot)
                             (CLOSED)
                             ))

;;making action
(setq *action-list*
      (list
       (instance pddl-action :init
                 :name "grasp-object"
                 :parameters '((?OBJ object))
                 :precondition '((NOT (ONHAND ?OBJ))
                                 (NOT (CLOSED))
                                 (AT PREGRASP))
                 :effect '((ONHAND ?OBJ)))

       (instance pddl-action :init
                 :name "grasp_f"
                 :parameters '((?OBJ object))
                 :precondition '((NOT (ONHAND ?OBJ))
                                 (NOT (CLOSED))
                                 (AT PREGRASP))
                 :effect '())

       (instance pddl-action :init
                 :name "move-to"
                 :parameters '((?FROM ?TO spot))
                 :precondition '((AT ?FROM)
                                 (NOT (= ?FROM SOMEWHERE))
                                 (NOT (= ?FROM ?TO)))
                 :effect '((AT ?TO)
                           (NOT (AT ?FROM))))
       (instance pddl-action :init
                 :name "move-to_f"
                 :parameters '((?FROM ?TO spot))
                 :precondition '((AT ?FROM)
                                 (NOT (= ?TO SOMEWHERE))
                                 (NOT (= ?FROM ?TO)))
                 :effect '((AT SOMEWHERE)
                           (NOT (AT ?FROM))))
       (instance pddl-action :init
                 :name "move-recoverly"
                 :parameters '()
                 :precondition '((AT SOMEWHERE)
                                 (CLOSED))
                 :effect '((AT FRONTFRIDGE)
                           (NOT (AT SOMEWHERE))))

       (instance pddl-action :init
                 :name "move_rec_f"
                 :parameters '()
                 :precondition '((AT SOMEWHERE)
                                 (CLOSED))
                 :effect '())

       (instance pddl-action :init
                 :name "open-door"
                 :parameters '()
                 :precondition '((AT FRONTFRIDGE)
                                 (NOT (ONHAND CAN))
                                 (CLOSED))
                 :effect '((NOT (CLOSED))))

       (instance pddl-action :init
                 :name "open_f"
                 :parameters '()
                 :precondition '((AT FRONTFRIDGE)
                                 (NOT (ONHAND CAN))
                                 (CLOSED))
                 :effect '())

       (instance pddl-action :init
                 :name "close-door"
                 :parameters '()
                 :precondition '((NOT (CLOSED))
                                 (AT PRECLOSE))
                 :effect '((CLOSED)))

       (instance pddl-action :init
                 :name "close_f"
                 :parameters '()
                 :precondition '((NOT (CLOSED))
                                 (AT PRECLOSE))
                 :effect '())

       (instance pddl-action :init
                 :name "try-close"
                 :parameters '()
                 :precondition '((NOT (CLOSED))
                                 (AT SOMEWHERE))
                 :effect '((CLOSED)))

       (instance pddl-action :init
                 :name "close_try_f"
                 :parameters '()
                 :precondition '((NOT (CLOSED))
                                 (AT SOMEWHERE))
                 :effect '())

       ))
;;add action to domain
(dolist (act *action-list*)
  (send *domain* :add :action act))

;;
;; solve planning
;;
(setq *failed-nodes* (list 'move-to))
(setq *graph*
      (pddl-plan-to-graph nil :domain *domain* :problem *problem* :failed-nodes *failed-nodes* :debug t))
(pprint *result*)

(when (and (ros::has-param "~display_graph")
           (ros::get-param "~display_graph"))
  ;;(piped-fork "gnome-open debug.pdf")
  (load "package://roseus_smach/src/pddl2smach.l")
  (load "package://roseus_smach/src/state-machine-ros.l")

  (setq *sm-inspect* (pddl-graph-to-smach *graph*))

  (unix::sleep 1)
  (send *sm-inspect* :reset-state)
 )

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
