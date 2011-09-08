#!/usr/bin/env roseus

(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

;; convert to smach
(load "package://roseus_smach/src/state-machine-ros.l")
(load "package://roseus_smach/src/state-machine-utils.l")
(load "package://roseus_smach/src/pddl2smach.l")

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
                 :name "put"
                 :parameters '((?OBJ ?BASE object))
                 :precondition '((GRASP ?OBJ))
                 :effect '((NOT (GRASP ?OBJ))
			   (ON ?OBJ ?BASE)))
       ))
(setq *failactlist1*
      (list
       (instance pddl-action :init
		 :name "put_f"
		 :parameters '((?OBJ ?BASE object))
		 :precondition '((GRASP ?OBJ))
		 :effect '())
       (instance pddl-action :init
		 :name "pick_f"
		 :parameters '((?OBJ object))
		 :precondition '((forall (?obj - object)
					 (NOT (GRASP ?obj))))
		 :effect '())
      ))
(setq *failactlist2*
      (list
       (instance pddl-action :init
		 :name "pick_f"
		 :parameters '((?OBJ object))
		 :precondition '((forall (?obj - object)
					 (NOT (GRASP ?obj))))
		 :effect '((ON ?OBJ FLOOR)))
       (instance pddl-action :init
		 :name "put_f"
		 :parameters '((?OBJ ?BASE object))
		 :precondition '((GRASP ?OBJ))
		 :effect '((NOT (GRASP ?OBJ))
			   (ON ?OBJ FLOOR)))
       ))

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
      '((ON CUP FLOOR)
        ))

;; we want to cup to be on table
(send *problem* :goal-condition
      '((ON CUP TABLE)))

(let ((ns (string-upcase (send *domain* :name))))
  (when (and ns (not (find-package ns)))
      (make-package ns)))

;;
;; solve planning
;;

(defun demo (&key (level 2))
  ;;add action to domain
  (dolist (ac *actlist*) (send *domain* :add :action ac))
  (cond ((= 1 level)
	 (dolist (ac *failactlist1*) (send *domain* :add :action ac)))
	((< 1 level)
	 (dolist (ac *failactlist2*) (send *domain* :add :action ac))))

  (pprint (setq *result* (solve-pddl-planning *domain* *problem* :debug nil)))

  ;; make graph from PDDL answer
  (setq *graph* (make-graph-from-pddl-results (list *result*) :node-name :pprint))

  ;; swap nodes ( check_open <-> check_open_f ) for making task flow
  (setq *graph* (add-failed-nodes *result* (list 'pick 'put) :graph *graph*))

  (send *graph* :name (send *domain* :name))

  (setq *graph* (make-readable-graph *graph*))
  (send *graph* :write-to-pdf "action_graph.pdf")
  (when (ros::get-param "~display_graph")
    (piped-fork "gnome-open action_graph.pdf"))

  ;; action definition
  ;; domain -> package
  ;; this file is created by (output-domain-template "pr2_action.l" *domain*)
  (cond ((< 0 level)
	 (defun PR2_ACTION::pick (obj)
	   (let ((val (random 2)))
	     (format t ";; move action [~a] -> ~a~%"
		     obj (if (= 0 val) "fail" "success"))
	     (if (= 0 val) nil t)))
	 (defun PR2_ACTION::put (obj base)
	   (let ((val (random 2)))
	     (format t ";; knock action [~a ~a] -> ~a~%"
		     obj base (if (= 0 val) "fail" "success"))
	     (if (= 0 val) nil t))))
	(t
	 (defun PR2_ACTION::pick (obj)
	   (format t ";; move action [~a]~%" obj) t)
	 (defun PR2_ACTION::put (obj base)
	     (format t ";; knock action [~a ~a]~%" obj base) t)))

  ;; global data is not used (nil)
  (exec-smach-with-spin (convert-smach *graph*) nil :hz 1.0)

  (when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
    (ros::exit))
  )
