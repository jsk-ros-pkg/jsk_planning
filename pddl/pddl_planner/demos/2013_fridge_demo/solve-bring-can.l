#!/usr/bin/env roseus

(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

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
                 :name "grasp"
                 :parameters '((?OBJ object))
                 :precondition '((NOT (ONHAND ?OBJ))
                                 (NOT (CLOSED))
                                 (AT PREGRASP))
                 :effect '((ONHAND ?OBJ)))
#|
       (instance pddl-action :init
                 :name "grasp_f"
                 :parameters '((?OBJ object))
                 :precondition '((NOT (ONHAND ?OBJ))
                                 (NOT (CLOSED))
                                 (AT PREGRASP))
                 :effect '())
|#
       (instance pddl-action :init
                 :name "move"
                 :parameters '((?FROM ?TO spot))
                 :precondition '((AT ?FROM)
                                 (NOT (= ?FROM SOMEWHERE))
                                 (NOT (= ?FROM ?TO)))
                 :effect '((AT ?TO)
                           (NOT (AT ?FROM))))
       (instance pddl-action :init
                 :name "move_f"
                 :parameters '((?FROM ?TO spot))
                 :precondition '((AT ?FROM)
                                 (NOT (= ?TO SOMEWHERE))
                                 (NOT (= ?FROM ?TO)))
                 :effect '((AT SOMEWHERE)
                           (NOT (AT ?FROM))))
       (instance pddl-action :init
                 :name "move_rec"
                 :parameters '()
                 :precondition '((AT SOMEWHERE)
                                 (CLOSED))
                 :effect '((AT FRONTFRIDGE)
                           (NOT (AT SOMEWHERE))))
#|
       (instance pddl-action :init
                 :name "move_rec_f"
                 :parameters '()
                 :precondition '((AT SOMEWHERE)
                                 (CLOSED))
                 :effect '())
|#
       (instance pddl-action :init
                 :name "open"
                 :parameters '()
                 :precondition '((AT FRONTFRIDGE)
                                 (NOT (ONHAND CAN))
                                 (CLOSED))
                 :effect '((NOT (CLOSED))))
#|
       (instance pddl-action :init
                 :name "open_f"
                 :parameters '()
                 :precondition '((AT FRONTFRIDGE)
                                 (NOT (ONHAND CAN))
                                 (CLOSED))
                 :effect '())
|#
       (instance pddl-action :init
                 :name "close"
                 :parameters '()
                 :precondition '((NOT (CLOSED))
                                 (AT PRECLOSE))
                 :effect '((CLOSED)))
#|
       (instance pddl-action :init
                 :name "close_f"
                 :parameters '()
                 :precondition '((NOT (CLOSED))
                                 (AT PRECLOSE))
                 :effect '())
|#
       (instance pddl-action :init
                 :name "close_try"
                 :parameters '()
                 :precondition '((NOT (CLOSED))
                                 (AT SOMEWHERE))
                 :effect '((CLOSED)))
#|
       (instance pddl-action :init
                 :name "close_try_f"
                 :parameters '()
                 :precondition '((NOT (CLOSED))
                                 (AT SOMEWHERE))
                 :effect '())
|#
       ))
;;add action to domain
(dolist (act *action-list*)
  (send *domain* :add :action act))


;;
;; solve planning
;;
(pprint (setq *result* (solve-pddl-planning *domain* *problem* :debug nil)))

(setq gr (make-graph-from-pddl-results (list *result*) :node-name :pprint))

(setq *graph* (add-failed-nodes *result* (list 'move 'close 'open 'grasp)))
;; swap nodes ( confirm <-> confirm_f and checkclear <-> checkclear_f ) for making task flow

(send *graph* :write-to-pdf "kakiuchi_robomec.pdf")

(when (and (ros::has-param "~display_graph")
           (ros::get-param "~display_graph"))
  (piped-fork "gnome-open kakiuchi_robomec.pdf")
  (load "package://roseus_smach/src/pddl2smach.l")
  (setq sm (convert-smach *graph*))

  (load "package://roseus_smach/src/state-machine-ros.l")
  (setq sm-inspect (instance state-machine-inspector :init sm))
  (unix::sleep 1)
  (send sm :reset-state)
  (progn
    (send sm-inspect :publish-structure)
    (send sm-inspect :publish-status nil))
  )

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
