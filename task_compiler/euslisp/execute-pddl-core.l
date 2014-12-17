#!/usr/bin/env roseus

(load "package://pddl_planner/src/eus-pddl-client.l")
(load "package://pddl_planner/src/pddl-result-graph.l")

;; convert to smach
(load "package://roseus_smach/src/state-machine-ros.l")
(load "package://roseus_smach/src/state-machine-utils.l")
(load "package://roseus_smach/src/pddl2smach.l")

;;
;; solve planning
;;
(defun run-pddl (&key action description goal-condition
                      (return-success :success) (return-fail :fail)
                      (add-self-return-failure-path t))
  (let (result problem graph act-names)
    ;; load action-functions, *domain*, *problem*
    (ros::ros-info "load action file [~a]" action)
    (load action)
    (ros::ros-info "load domain and problem file [~a]" description)
    (load description)

    (setq problem *problem*)
    (when goal-condition
      (setq problem (copy-object *problem*))
      (send problem :goal-condition goal-condition)
      )
    ;; solve the pddl problem and make graph from PDDL answer
    (if *failed-nodes* (ros::ros-info "found failed-nodes: ~A" *failed-nodes*))
    (setq graph (pddl-plan-to-graph nil :domain *domain* :problem problem
                                    :failed-nodes (if *failed-nodes*
                                                      *failed-nodes* nil)
                                    :debug nil))

#|
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
|#
    (send graph :name (send *domain* :name))
    (setq graph (make-readable-graph graph))

    (send graph :write-to-pdf "action_graph.pdf")
    (unix::system "mv action_graph.pdf /tmp/")
    (ros::ros-info "output result to /tmp/action_graph.pdf")

    (when (ros::get-param "~display_graph")
      (piped-fork "gnome-open /tmp/action_graph.pdf"))

    (setq *sm* (convert-smach graph
                :return-success return-success :return-fail return-fail
                :add-self-return-failure-path add-self-return-failure-path
                ;; :use-userdata t
                ))
    (dump-loadable-structure "/tmp/action_state_machine.l" *sm*)
    (ros::ros-info "output state-machine *sm* to /tmp/action_state_machine.l"))

     ;; global data is not used (nil)
    (exec-smach-with-spin *sm* nil :hz 1.0)

    (when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
      (ros::exit))
    )

