#!/usr/bin/env roseus

(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'pr2_action))
(send *domain* :requirements '(:typing))
(send *domain* :types '(object))
(send *domain* :predicates
      '((ON ?OBJ1 ?OBJ2 - object)
        (GRASP ?OBJ - object)))
;;making action
(setq *actlist*
      (list
       (instance pddl-action :init
                 :name "pick"
                 :parameters '((?OBJ object))
                 :precondition '((forall (?obj - object)
                                         (NOT (GRASP ?obj))))
                 :effect '((GRASP ?OBJ)
			   (forall (?base - object)
				   (NOT (ON ?OBJ ?base)))))
       (instance pddl-action :init
                 :name "pick_f"
                 :parameters '((?OBJ object))
                 :precondition '((forall (?obj - object)
                                         (NOT (GRASP ?obj))))
                 :effect '((ON ?OBJ FLOOR)))
;                 :effect '())
       (instance pddl-action :init
                 :name "put"
                 :parameters '((?OBJ ?BASE object))
                 :precondition '((GRASP ?OBJ))
                 :effect '((NOT (GRASP ?OBJ))
			   (ON ?OBJ ?BASE)))
       (instance pddl-action :init
                 :name "put_f"
                 :parameters '((?OBJ ?BASE object))
                 :precondition '((GRASP ?OBJ))
                 :effect '((NOT (GRASP ?OBJ))
			   (ON ?OBJ FLOOR)))
;                 :effect '())
       ))

;;add action to domain
(dolist (ac *actlist*)
  (send *domain* :add :action ac))

;;
;; problem
;;
;; failure recovery example task
(setq *problem*
      (instance pddl-problem :init :name 'failure-recovery-task :domain 'pr2_action))

;; three objects
(send *problem* :objects
      '((CUP . object)
        (TABLE . object)
        (FLOOR . object)
      ))

;; initial condition
(send *problem* :initial-condition
      '(;; from sensor-data or heuristics
        (ON CUP FLOOR)
        ))

;;
;; solve planning
;;

;; we want to enter a room
(send *problem* :goal-condition
      '((ON CUP TABLE)))
(pprint (setq *result1* (solve-pddl-planning *domain* *problem* :debug nil)))

(setq results (list *result1*))

;; make graph from PDDL answer
(setq *graph* (make-graph-from-pddl-results results :node-name :pprint))
;; swap nodes ( check_open <-> check_open_f ) for making task flow

(dolist (res results)
  (setq *graph* (add-failed-nodes res (list 'pick 'put) :graph *graph*)))

(send *graph* :name (send *domain* :name))

(setq *graph* (make-readable-graph *graph*))
(send *graph* :write-to-pdf "action-graph.pdf")
(when (ros::get-param "~display_graph")
  (piped-fork "gnome-open action-graph.pdf"))

;; action definition
;; domain -> package
;; this file is created by (output-domain-template "pr2_action.l" *domain*)
(let ((ns (string-upcase (send *domain* :name))))
  (when (and ns (not (find-package ns)))
      (make-package ns)))
(defun PR2_ACTION::pick (obj)
  (let ((val (random 2)))
    (format t ";; move action [~a] -> ~a~%"
	    obj (if (= 0 val) "fail" "success"))
    (if (= 0 val) nil t)))
(defun PR2_ACTION::put (obj base)
  (let ((val (random 2)))
    (format t ";; knock action [~a ~a] -> ~a~%"
	    obj base (if (= 0 val) "fail" "success"))
    (if (= 0 val) nil t)))

;; convert to smach
(load "package://roseus_smach/src/state-machine-ros.l")
(load "package://roseus_smach/src/state-machine-utils.l")
(load "package://roseus_smach/src/pddl2smach.l")

;; global data is not used (nil)
(exec-smach-with-spin (convert-smach *graph*) nil :hz 1.0)

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
