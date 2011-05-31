#!/usr/bin/env roseus

(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

;;
;; problem
;;
;; mikita knock
(setq *problem*
      (instance pddl-problem :init :name 'mikita-knock :domain 'mikita-1week))

;; one object and three spots
(send *problem* :objects
      '((DOOR . object)
        (DOORFRONT . spot)
        (ROOMCENTER . spot)
        (STARTSPOT . spot))
      )

;; initial condition
;; door sepalates {START,DOORFRONT} and ROOM
(send *problem* :initial-condition
      '((NOT (OPEN DOOR))
	(AT STARTSPOT)
	(REACHABLE DOOR DOORFRONT)
	(SEPALATE DOOR STARTSPOT ROOMCENTER)
	(SEPALATE DOOR DOORFRONT ROOMCENTER)
	))

;; we want to enter a room
(send *problem* :goal-condition
      '((AT ROOMCENTER)
      ))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'mikita-1week))
(send *domain* :requirements '(:typing))
(send *domain* :types '(object spot))
(send *domain* :predicates '((AT ?SPOT - spot)
			     (REACHABLE ?OBJ - object ?SPOT - spot) ;; not used
			     (OPEN ?OBJ - object)
			     (SEPALATE ?OBJ - object ?FROM ?TO - spot)
			     ))
;;making action
(setq *actlist*
      (list
       (instance pddl-action :init
                 :name "move"
                 :parameters '((?FROM ?TO spot))
                 :precondition '((AT ?FROM)
                                 (NOT (= ?FROM ?TO))
				 (NOT (exists (?obs - object)
					     (SEPALATE ?obs ?FROM ?TO))))
                 :effect '((AT ?TO)
                           (NOT (AT ?FROM))))

       (instance pddl-action :init
		 :name "open_action"
		 :parameters '((?OBJ object) (?SPOT spot))
		 :precondition '((AT ?SPOT)
				 (REACHABLE ?OBJ ?SPOT))
		 :effect '((OPEN ?OBJ)))

       (instance pddl-action :init
		 :name "check_open"
		 :parameters '((?OBJ object) (?SPOT spot))
		 :precondition '((REACHABLE ?OBJ ?SPOT)
				 (OPEN ?OBJ))
		 :effect '((forall (?S1 ?S2 - spot)
				   (NOT (SEPALATE ?OBJ ?S1 ?S2)))
			   ))
       (instance pddl-action :init
		 :name "check_open_f"
		 :parameters '((?OBJ object) (?SPOT spot))
		 :precondition '((REACHABLE ?OBJ ?SPOT)
				 (OPEN ?OBJ))
		 :effect '((NOT (OPEN ?OBJ))))

       ))

;;add action to domain
(dolist (ac *actlist*)
  (send *domain* :add :action ac))

;;
;; solve planning
;;
(pprint (setq *result* (solve-pddl-planning *domain* *problem*)))

(setq gr (make-graph-from-pddl-results (list *result*) :node-name :pprint))

(setq *graph* (add-failed-nodes *result* (list 'check_open)))
;; swap nodes ( check_open <-> check_open_f ) for making task flow

(send *graph* :write-to-pdf "knock_demo.pdf")

(when (ros::get-param "~display_graph")
  (piped-fork "gnome-open knock_demo.pdf")

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))




