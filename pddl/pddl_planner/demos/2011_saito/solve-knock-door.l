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
	(TRASHBOXFRONT . spot)
        (STARTSPOT . spot))
      )

;; initial condition
;; door sepalates {START,DOORFRONT} and ROOM
(send *problem* :initial-condition
      '((NOT (OPEN DOOR))
	(AT STARTSPOT)
	(REACHABLE DOOR DOORFRONT)
	(SEPARATE DOOR STARTSPOT ROOMCENTER)
	(SEPARATE DOOR DOORFRONT ROOMCENTER)
	(SEPARATE DOOR TRASHBOXFRONT ROOMCENTER)
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
			     (SEPARATE ?OBJ - object ?FROM ?TO - spot)
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
					     (SEPARATE ?obs ?FROM ?TO))))
                 :effect '((AT ?TO)
                           (NOT (AT ?FROM))))

       (instance pddl-action :init
		 :name "open_action"
		 :parameters '((?OBJ object) (?SPOT spot))
		 :precondition '((AT ?SPOT)
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

       ))

;;add action to domain
(dolist (ac *actlist*)
  (send *domain* :add :action ac))

;;
;; solve planning
;;

;; we want to enter a room
(send *problem* :goal-condition
      '((AT ROOMCENTER)))
(pprint (setq *result1* (solve-pddl-planning *domain* *problem* :debug nil)))

;; we want to go to in front of a trash box
(send *problem* :goal-condition
      '((AT TRASHBOXFRONT)))
(pprint (setq *result2* (solve-pddl-planning *domain* *problem* :debug nil)))

(setq gr (make-graph-from-pddl-results (list *result1* *result2*) :node-name :pprint))

(setq *graph* (add-failed-nodes *result* (list 'check_open)))
;; swap nodes ( check_open <-> check_open_f ) for making task flow

(send *graph* :write-to-pdf "knock_demo.pdf")

;(when (ros::get-param "~display_graph")
;  (piped-fork "gnome-open knock_demo.pdf"))

;; action definition
(defun move (from to)
  (format t ";; move action [~a ~a]~%" from to) t)
(defun open_action (obj spot)
  (format t ";; open action [~a ~a]~%" obj spot) t)
(defun check_open (obj spot)
  (format t ";; check if open [~a ~a]~%" obj spot)
  (< 0.8 (rand))) ;; 20% success

;; convert to smach
(load "package://roseus_smach/src/state-machine-ros.l")
(load "convert-smach.l")
(setq sm (convert-smach *graph*) mydata nil) ;; global data is not used
(setq insp (instance state-machine-inspector :init sm))
(unix:sleep 1) ;; for connection
(send sm :reset-state) ;; ?
(send insp :publish-structure)
(send insp :publish-status mydata)
(while (not (send sm :goal-reached))
  (unix:sleep 1)
  (send insp :publish-status mydata)
  ;; you can execute state-machine step by step
  (send sm :execute mydata :step -1))

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
