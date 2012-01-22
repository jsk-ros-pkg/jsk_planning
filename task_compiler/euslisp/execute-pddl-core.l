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
    (ros::ros-info "load action file [~a]" action)
    (load action)
    (ros::ros-info "load domain and problem file [~a]" description)
    (load description)

    ;; solve the pddl problem
    (setq result (solve-pddl-planning *domain* *problem* :debug nil))
    ;; (pprint result)

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
    (when act-names
      (setq graph (add-failed-nodes result act-names :graph graph)))

    (send graph :name (send *domain* :name))
    (setq graph (make-readable-graph graph))

    (send graph :write-to-pdf "action_graph.pdf")
    (unix::system "mv action_graph.pdf /tmp/")
    (ros::ros-info "output result to /tmp/action_graph.pdf")

    (when (ros::get-param "~display_graph")
      (piped-fork "gnome-open /tmp/action_graph.pdf"))

    (setq *sm* (convert-smach graph))
    (dump-loadable-structure "/tmp/action_state_machine.l" *sm*)
    (ros::ros-info "output state-machine *sm* to /tmp/action_state_machine.l"))

     ;; global data is not used (nil)
    (exec-smach-with-spin *sm* nil :hz 1.0)

    (when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
      (ros::exit))
    )

