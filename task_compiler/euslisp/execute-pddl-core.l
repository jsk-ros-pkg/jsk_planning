#!/usr/bin/env roseus

(load "package://pddl_planner/src/pddl-result-graph.l")

;; convert to smach
(load "package://roseus_smach/src/state-machine-ros.l")
(load "package://roseus_smach/src/state-machine-utils.l")
(load "package://roseus_smach/src/pddl2smach.l")

;;
;; solve planning
;;
(defun run-pddl (&key action description)
  (let (result graph act-names)
    ;; load action-functions, *domain*, *problem*
    (load action)
    (load description)

    ;; solve the pddl problem
    (setq result (solve-pddl-planning *domain* *problem* :debug nil))
    (pprint result)

    ;; make graph from PDDL answer
    (setq graph (make-graph-from-pddl-results (list result) :node-name :pprint))

    ;; swap nodes ( check_open <-> check_open_f ) for making task flow
    (setq act-names (mapcar #'string-upcase
			    (send-all (send *domain* :action) :name)))
    (setq act-names
	  (mapcan
	   #'(lambda(s)
	       (if (find (concatenate string s "_F") act-names :test #'string=)
		   (list (intern s))))
	   act-names))
    (setq graph (add-failed-nodes result act-names :graph graph))

    (send graph :name (send *domain* :name))
    (setq graph (make-readable-graph graph))

    (when (ros::get-param "~display_graph")
      (send graph :write-to-pdf "action_graph.pdf")
      (piped-fork "gnome-open action_graph.pdf"))

    ;; global data is not used (nil)
    (exec-smach-with-spin (convert-smach graph) nil :hz 1.0)

    (when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
      (ros::exit))
    ))
