#!/usr/bin/env roseus

(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'pr2_action))
(send *domain* :requirements '(:typing))
(send *domain* :types '(object spot))
(send *domain* :predicates
	  '(
		(AT ?SPOT - spot) ;; the spot which robot is in
		(ENABLE ?OBJ - object) ;; the object is enable (door -> close)
		(CLEAN ?OBJ - object)
		(GRASP ?OBJ - object)
		(ON ?OBJ ?BASE - object) ;; the current position of OBJ
		(STRAGE ?OBJ ?BASE - object) ;; proper position of OBJ
		(REACHABLE ?OBJ - object ?SPOT - spot)
		(SALE-AT ?OBJ - object ?SPOT - spot)
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
						 (forall (?obs - object)
								 (NOT (SEPARATE ?obs ?FROM ?TO))))
		 :effect '((AT ?TO)
				   (NOT (AT ?FROM))))

       (instance pddl-action :init
		 :name "pick"
		 :parameters '((?OBJ object))
		 :precondition '((exists (?base - object ?spot - spot)
								 (AND (ON ?OBJ ?base)
									  (REACHABLE ?base ?spot)
									  (AT ?spot)))
						 (forall (?objx - object)
								  (NOT (GRASP ?objx)))
						 )
		 :effect '((GRASP ?OBJ)
				   (forall (?base - object)
						   (NOT (ON ?OBJ ?base)))
				   ))
       (instance pddl-action :init
		 :name "put"
		 :parameters '((?OBJ ?BASE object))
		 :precondition '((exists (?spot - spot)
								 (AND (REACHABLE ?BASE ?spot)
									  (AT ?spot)))
						 (GRASP ?OBJ))
		 :effect '((NOT (GRASP ?OBJ))
				   (ON ?OBJ ?BASE)))

	   (instance pddl-action :init
		 :name "buy"
		 :parameters '((?OBJ object))
		 :precondition '((exists (?spot - spot)
								 (AND (SALE-AT ?OBJ ?spot)
									  (AT ?spot))))
		 :effect '((GRASP ?OBJ)))


       (instance pddl-action :init
		 :name "wipe"
		 :parameters '((?OBJ object))
		 :precondition '((exists (?spot - spot)
								 (AND (REACHABLE ?OBJ ?spot)
									  (AT ?spot)))
						 (forall (?gomi - object)
								  (NOT (ON ?gomi ?OBJ)))
						 (NOT (CLEAN ?OBJ)))
		 :effect '((CLEAN ?OBJ)))

       (instance pddl-action :init
		 :name "open"
		 :parameters '((?OBJ object))
		 :precondition '((exists (?spt - spot)
								 (AND (REACHABLE ?OBJ ?spt)
									  (AT ?spt)))
		                 (ENABLE ?OBJ))
		 :effect '((NOT (ENABLE ?OBJ))))

       (instance pddl-action :init
		 :name "check_open"
		 :parameters '((?OBJ object))
		 :precondition '((exists (?spot - spot)
								 (AND (REACHABLE ?OBJ ?spot)
									  (AT ?spot)))
						 (NOT (ENABLE ?OBJ)))
		 :effect '((forall (?S1 ?S2 - spot)
						   (NOT (SEPARATE ?OBJ ?S1 ?S2)))
				   ))
       (instance pddl-action :init
		 :name "check_open_f"
		 :parameters '((?OBJ object))
		 :precondition '((exists (?spot - spot)
								 (AND (REACHABLE ?OBJ ?spot)
									  (AT ?spot)))
						 (NOT (ENABLE ?OBJ)))
		 :effect '((ENABLE ?OBJ)))
       ))

;;add action to domain
(dolist (ac *actlist*)
  (send *domain* :add :action ac))

;;
;; problem
;;
;; mikita knock
(setq *problem*
      (instance pddl-problem :init :name 'mikita-knock :domain 'pr2_action))

;; one object and three spots
(send *problem* :objects
      '((RM73B2-DOOR . object)
		(RM73B2-TABLE . object)
		(PLASTIC-BOTTLE . object)
		(TRASHBOX . object)
		(SAITO-TABLE . object)
		(SANDWITCH . object)
		(RM73B2-DOORFRONT . spot)
        (RM73B2-TABLEFRONT . spot)
		(TRASHBOXFRONT . spot)
		(SHOPFRONT . spot)
        (RM73A3-CENTER . spot))
      )

;; initial condition
(send *problem* :initial-condition
      '(;; from sensor-data or heuristics
		(AT RM73A3-CENTER)
		(ENABLE RM73B2-DOOR)
		;; generate from knowledge-base/manip-planner
		(ON PLASTIC-BOTTLE RM73B2-TABLE)
		(REACHABLE RM73B2-DOOR RM73B2-DOORFRONT)
		(REACHABLE RM73B2-TABLE RM73B2-TABLEFRONT)
		(REACHABLE TRASHBOX TRASHBOXFRONT)
		(REACHABLE SAITO-TABLE RM73A3-CENTER)
		(SALE-AT SANDWITCH SHOPFRONT)
		;; generate fron jsk_map
		(SEPARATE RM73B2-DOOR RM73A3-CENTER RM73B2-TABLEFRONT)
		(SEPARATE RM73B2-DOOR RM73B2-DOORFRONT RM73B2-TABLEFRONT)
		(SEPARATE RM73B2-DOOR TRASHBOXFRONT RM73B2-TABLEFRONT)
		(SEPARATE RM73B2-DOOR SHOPFRONT RM73B2-TABLEFRONT)
		))

;;
;; solve planning
;;

;; we want to enter a room
(send *problem* :goal-condition
      '((CLEAN RM73B2-TABLE)
		(AT RM73A3-CENTER)))
(pprint (setq *result1* (solve-pddl-planning *domain* *problem* :debug nil)))

;; we want to go to in front of a trash box
(send *problem* :goal-condition
      '((ON PLASTIC-BOTTLE TRASHBOX)
		(AT RM73A3-CENTER)))
(pprint (setq *result2* (solve-pddl-planning *domain* *problem* :debug nil)))

(send *problem* :goal-condition
      '((ON SANDWITCH SAITO-TABLE)
		(CLEAN SAITO-TABLE)
		(AT RM73A3-CENTER)))
(pprint (setq *result3* (solve-pddl-planning *domain* *problem* :debug nil)))

(setq results (list *result1* *result2* *result3*))

;; make graph from PDDL answer
(setq *graph* (make-graph-from-pddl-results results :node-name :pprint))
;; swap nodes ( check_open <-> check_open_f ) for making task flow
(dolist (res results)
  (setq *graph* (add-failed-nodes res (list 'check_open) :graph *graph*)))

(send *graph* :name (send *domain* :name))

;(send *graph* :write-to-pdf "knock_demo.pdf")
;(when (ros::get-param "~display_graph")
;  (piped-fork "gnome-open knock_demo.pdf"))

;; action definition
;; domain -> package
;; this file is created by (output-domain-template "pr2_action.l" *domain*)
(let ((ns (string-upcase (send *domain* :name))))
  (when (and ns (not (find-package ns)))
      (make-package ns)))
(defun PR2_ACTION::move (from to)
  (format t ";; move action [~a ~a]~%" from to) t)
(defun PR2_ACTION::open (obj)
  (format t ";; open action [~a]~%" obj) t)
(defun PR2_ACTION::check_open (obj)
  (format t ";; check if open [~a]~%" obj)
  (< 0.8 (rand))) ;; 20% success
(defun PR2_ACTION::wipe (obj)
  (format t ";; wiping [~a]       ~%" obj) t)
(defun PR2_ACTION::pick (obj)
  (format t ";; picking [~a]      ~%" obj) t)
(defun PR2_ACTION::put (obj base)
  (format t ";; put [~a -> ~a]    ~%" obj base) t)
(defun PR2_ACTION::buy (obj)
  (format t ";; buy [~a at ~a]    ~%" obj) t)

;; convert to smach
(load "package://roseus_smach/src/state-machine-ros.l")
(load "package://roseus_smach/src/state-machine-utils.l")
(load "package://roseus_smach/src/pddl2smach.l")

;; global data is not used (nil)
(exec-smach-with-spin (convert-smach *graph*) nil :hz 1.0)

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
