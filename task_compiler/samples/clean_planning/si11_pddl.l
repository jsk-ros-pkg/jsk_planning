#!/usr/bin/env roseus

(load "package://pddl_planner/src/eus-pddl-client.l")

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'pr2_action))
(send *domain* :requirements '(:typing))
(send *domain* :types '(item base spot arm))
(send *domain* :predicates
      '(
        (AT ?SPOT - spot) ;; the spot which robot is in
        (ON ?OBJ - item ?BASE - base)
        (SPOT-ON ?OBJ - item ?SPOT - spot)
        (CLEAN ?OBJ - base)
        (REACHABLE ?OBJ - base ?SPOT - spot)
	(REACHABLE-SS ?SPOT1 - spot ?SPOT2 - spot)
	(ARM-GRASP ?OBJ - item ?ARM - arm)
	(TYPICALLY-AT ?OBJ - item ?SPOT - spot)
        ))
;;
;;action
;;
(setq *actlist*
      (list
       (instance pddl-action :init
                 :name "move-to-room"
                 :parameters '((?TO spot))
                 :precondition '((NOT (AT ?TO))
				 (NOT (exists (?OBJ - item)
					      (SPOT-ON ?OBJ ?TO))))
                 :effect '((AT ?TO)
			   (forall (?FROM - spot)
				   (when (NOT (= ?TO ?FROM))
					 (NOT (AT ?FROM))))))
       (instance pddl-action :init
                 :name "move-chair"
                 :parameters '((?OBJ item) (?TOON spot))
                 :precondition '((exists (?FROM - spot)
					 (exists (?FROMON - spot)
						 (AND (SPOT-ON ?OBJ ?FROMON)
						      (REACHABLE-SS ?FROMON ?FROM)
						      (AT ?FROM))))
				 (TYPICALLY-AT ?OBJ ?TOON))
		 :effect '((SPOT-ON ?OBJ ?TOON)
			   (forall (?SPOT - spot)
				   (when (NOT (= ?TOON ?SPOT))
				     (NOT (SPOT-ON ?OBJ ?SPOT))))
			   (forall (?SPOT - spot)
				   (when (AT ?SPOT)
				     (REACHABLE-SS ?TOON ?SPOT)))
			   (forall (?SPOT - spot)
				   (when (NOT (AT ?SPOT))
				     (NOT (REACHABLE-SS ?TOON ?SPOT)))))
		 )
       (instance pddl-action :init
                 :name "pick"
                 :parameters '((?OBJ item) (?ARM arm))
                 :precondition '((exists (?SPOT - spot)
					 (exists (?BASE - base)
						 (AND (AT ?SPOT)
						      (ON ?OBJ ?BASE)
						      (REACHABLE ?BASE ?SPOT))))
                                 (NOT (exists (?objx - item)
					     (ARM-GRASP ?objx ?ARM))))
                 :effect '((ARM-GRASP ?OBJ ?ARM)
                           (forall (?BASE - base)
                                   (NOT (ON ?OBJ ?BASE)))
                           ))
       (instance pddl-action :init
                 :name "release"
                 :parameters '((?BASE base) (?ARM arm))
                 :precondition '((exists (?SPOT - spot)
                                         (AND (AT ?SPOT)
					      (REACHABLE ?BASE ?SPOT)))
                                 (exists (?OBJ - item)
                                         (ARM-GRASP ?OBJ ?ARM)))
                 :effect '((forall (?OBJ - item)
				   (when (ARM-GRASP ?OBJ ?ARM)
				     (AND (NOT (ARM-GRASP ?OBJ ?ARM))
					  (ON ?OBJ ?BASE))))
                           ))
       (instance pddl-action :init
                 :name "wipe-table4"
                 :parameters '((?BASE base))
                 :precondition '((exists (?SPOT - spot)
                                         (AND (REACHABLE ?BASE ?SPOT)
                                              (AT ?SPOT)))
				 (forall (?OBJ - item)
					 (NOT (ON ?OBJ ?BASE)))
				 (forall (?ARM - arm)
					 (NOT (exists (?OBJ - item)
						     (ARM-GRASP ?OBJ ?ARM))))
                                 (NOT (CLEAN ?BASE)))
                 :effect '((CLEAN ?BASE)))
       ))

;;add action to domain
(dolist (ac *actlist*)
  (send *domain* :add :action ac))

;;
;; problem
;;
;; mikita SI11
(setq *problem*
      (instance pddl-problem :init :name 'mikita-si11 :domain 'pr2_action))

;; one object and three spots
(send *problem* :objects
      '((KITCHEN-SINK . base)
	(TRASHBOX . base)
	(TABLE . base)
        (SUSHI-CUP2 . item)
	(EMPTY-BOX . item)
	(BOTTLE . item)
	(CHAIR1 . item)
	(START-SPOT . spot)
        (TABLE-FRONT . spot)
        (TRASHBOX-FRONT . spot)
        (SINK-FRONT . spot)
        (CHAIR-BACK . spot)
        (TMP-CHAIR-SPOT . spot)
	(L-ARM . arm)
	(R-ARM . arm))
      )

;; initial condition
(send *problem* :initial-condition
      '(;; from sensor-data or heuristics
        (AT START-SPOT)
	;;
	(ON SUSHI-CUP2 TABLE)
	(ON EMPTY-BOX TABLE)
	(ON BOTTLE TABLE)
	(SPOT-ON CHAIR1 TABLE-FRONT)
        ;; generate from knowledge-base/manip-planner
        (REACHABLE TABLE TABLE-FRONT)
        (REACHABLE TRASHBOX TRASHBOX-FRONT)
        (REACHABLE KITCHEN-SINK SINK-FRONT)
	(REACHABLE-SS TABLE-FRONT CHAIR-BACK)
	(TYPICALLY-AT CHAIR1 TABLE-FRONT)
	(TYPICALLY-AT CHAIR1 TMP-CHAIR-SPOT)
        ))

;;
;; solve planning
;;

;; we want to enter a room
(send *problem* :goal-condition
      '((CLEAN TABLE)
	(ON SUSHI-CUP2 KITCHEN-SINK)
	(ON EMPTY-BOX TRASHBOX)
	(ON BOTTLE TRASHBOX)
	(SPOT-ON CHAIR1 TABLE-FRONT)
	))

(defun debug-planning-result ()
  (unless (classp pddl-graph) ;; TODO (require,provide)
    (load "package://pddl_planner/src/pddl-result-graph.l"))
(pprint (setq *result1* (solve-pddl-planning *domain* *problem* :debug nil)))

;(setq results (list *result1* *result2* *result3*))
(setq results (list *result1*))

;; make graph from PDDL answer
(setq *graph* (make-graph-from-pddl-results results :node-name :pprint))
;; swap nodes ( check_open <-> check_open_f ) for making task flow

;(dolist (res results)
;  (setq *graph* (add-failed-nodes res (list 'knock 'wipe 'move) :graph *graph*)))

(send *graph* :name (send *domain* :name))

(send (make-readable-graph *graph*) :write-to-pdf "knock_demo.pdf")
;(when (ros::get-param "~display_graph")
  (piped-fork "gnome-open knock_demo.pdf")
;)

;; action definition
;; domain -> package
;; this file is created by (output-domain-template "pr2_action.l" *domain*)
;(let ((ns (string-upcase (send *domain* :name))))
;  (when (and ns (not (find-package ns)))
;      (make-package ns)))
;(defun PR2_ACTION::move (from to)
;  (format t ";; move action [~a ~a]~%" from to) t)
;(defun PR2_ACTION::knock (obj)
;  (format t ";; knock action [~a]~%" obj) t)
;(defun PR2_ACTION::wipe (obj)
;  (format t ";; wiping [~a]       ~%" obj) t)

;; convert to smach
;(load "package://roseus_smach/src/state-machine-ros.l")
;(load "package://roseus_smach/src/state-machine-utils.l")
;(load "package://roseus_smach/src/pddl2smach.l")

;; global data is not used (nil)
;(exec-smach-with-spin (convert-smach *graph*) nil :hz 1.0)

;(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
;  (ros::exit))
)