#!/usr/bin/env roseus

(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

;;
;; problem
;;
(setq *problem*
      (instance pddl-problem :init :name 'with-elevator :domain 'navigation))

;; one object and spots
(send *problem* :objects
      '((STARTSPOT . spot)
	(GOALSPOT . spot)
     	(STARTFLOOR . floor)
	(GOALFLOOR . floor)
	(ENG2-2F . floor)         ;; from map
	(ENG2-7F . floor)         ;; from map
	(ENG2-2F-ELEVATOR-DOOR . obstacle)    ;; from map
	(ENG2-2F-ELEVATOR-CALLBTN . button)   ;; from map
	(ENG2-2F-ELEVATOR-INSIDEBTN . button) ;; from map
	(ENG2-7F-ELEVATOR-DOOR . obstacle)    ;; from map
	(ENG2-7F-ELEVATOR-CALLBTN . button)   ;; from map
	(ENG2-7F-ELEVATOR-INSIDEBTN . button) ;; from map
        (ENG2-2F-CALLPANELFRONT . spot)   ;; from map
        (ENG2-2F-INSIDEPANELFRONT . spot) ;; from map
	(ENG2-2F-ELEVATORFRONT . spot)    ;; from map
        (ENG2-7F-CALLPANELFRONT . spot)   ;; from map
        (ENG2-7F-INSIDEPANELFRONT . spot) ;; from map
	(ENG2-7F-ELEVATORFRONT . spot)    ;; from map
	)
      )

;; initial condition
(send *problem* :initial-condition
      '((SPOT-AT STARTSPOT)
	;; from map
	(REACHABLE-SPOT ENG2-2F-CALLPANELFRONT ENG2-2F-ELEVATOR-CALLBTN)
	(REACHABLE-SPOT ENG2-7F-CALLPANELFRONT ENG2-7F-ELEVATOR-CALLBTN)
	(REACHABLE-SPOT ENG2-2F-INSIDEPANELFRONT ENG2-2F-ELEVATOR-INSIDEBTN)
	(REACHABLE-SPOT ENG2-7F-INSIDEPANELFRONT ENG2-7F-ELEVATOR-INSIDEBTN)
	(ASSOC ELEVATOR INSIDEPANELFRONT)
	))

;; we want to enter a room
(send *problem* :goal-condition
      '((SPOT-AT GOALSPOT)
      ))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'navigation))
(send *domain* :requirements '(:typing))
(send *domain* :types '(obstacle viehcle button spot floor))
(send *domain* :predicates '((SPOT-AT ?SPOT - spot)
			     (FLOOR-AT ?FLOOR - floor)
			     (REACHABLE-SPOT ?SPOT - spot ?BTN - button)
			     (OPEN ?OBJ - object)
			     (PUSHED ?OBJ - button)
			     (SEPARATE ?OBJ - object ?FROM ?TO - spot)
			     ))
;;making action
(setq *actlist*
      (list
       (instance pddl-action :init
                 :name "move"
                 :parameters '((?FROM ?TO spot))
                 :precondition '((SPOT-AT ?FROM)
                                 (NOT (= ?FROM ?TO))
				 (NOT (exists (?obs - object)
					     (SEPARATE ?obs ?FROM ?TO))))
                 :effect '((SPOT-AT ?TO)
                           (NOT (SPOT-AT ?FROM))))

       ;; for elevator move
       (instance pddl-action :init
		 :name "elevator_move"
		 :parameters '((?OBJ viehcle) (?SPOT spot) (?FROM ?TO floor))
		 :precondition '((SPOT-AT ?SPOT)
				 (ASSOC ?SPOT ?OBJ))
		 :effect '((OPEN ?OBJ)))

       ;; for door open
       (instance pddl-action :init
		 :name "open_action"
		 :parameters '((?OBJ object) (?SPOT spot))
		 :precondition '((SPOT-AT ?SPOT)
		                 (NOT (OPEN ?OBJ))
				 (REACHABLE ?OBJ ?SPOT)
                                 )
		 :effect '((OPEN ?OBJ)))
       (instance pddl-action :init
		 :name "check_open"
		 :parameters '((?OBJ object) (?SPOT spot))
		 :precondition '((REACHABLE ?OBJ ?SPOT)
				 (OPEN ?OBJ))
		 :effect '((forall (?S1 ?S2 - spot)
				   (NOT (SEPARATE ?OBJ ?S1 ?S2)))
			   ))
       (instance pddl-action :init
		 :name "check_open_f"
		 :parameters '((?OBJ object) (?SPOT spot))
		 :precondition '((REACHABLE ?OBJ ?SPOT)
				 (OPEN ?OBJ))
		 :effect '((NOT (OPEN ?OBJ))))

       ;; for push action
       (instance pddl-action :init
		 :name "push_action"
		 :parameters '((?OBJ button) (?SPOT spot))
		 :precondition '((SPOT-AT ?SPOT)
				 (REACHABLE ?OBJ ?SPOT)
                                 )
		 :effect '((OPEN ?OBJ)))
       (instance pddl-action :init
		 :name "check_pushed"
		 :parameters '((?OBJ button) (?SPOT spot))
		 :precondition '((REACHABLE ?OBJ ?SPOT)
				 (PUSHED ?OBJ))
		 :effect '((forall (?S1 ?S2 - spot)
				   (NOT (SEPARATE ?OBJ ?S1 ?S2)))
			   ))
       (instance pddl-action :init
		 :name "check_pushed_f"
		 :parameters '((?OBJ button) (?SPOT spot))
		 :precondition '((REACHABLE ?OBJ ?SPOT)
				 (PUSHED ?OBJ))
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
