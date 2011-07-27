(load "package://pddl_planner/src/eus-pddl.l")
(load "package://pddl_planner/src/ffha-result-parser.l")

;;
;; PDDL domain and problem should be define before using this program.
;;

;;
;; ROS
;;
(ros::roseus "pddl_client")

(setq *action-client*
      (instance ros::simple-action-client :init
                "pddl_planner" pddl_msgs::PDDLPlannerAction))

(defun pddl-planning (domain problem &key (client *action-client*) (debug))
  (let (plan ret)
    (warning-message 1 "waiting pddl_planner~%")
    (send client :wait-for-server)
    (warning-message 1 "get actionlib server~%")
    (let ((goal (instance pddl_msgs::PDDLPlannerActionGoal :init)))
      (send goal :goal :domain (send domain :ros-message))
      (send goal :goal :problem (send problem :ros-message))
      (send client :send-goal goal)
      (warning-message 1 "sent goal~%"))

    (warning-message 1 "waiting goal...~%")
    (send client :wait-for-result)

    (when (send (send client :get-result) :sequence)
      (setq plan (mapcar #'(lambda (x)
                             (cons (read-from-string (remove #\( (send x :action)))
                                   (mapcar #'(lambda (y) (read-from-string (remove #\) y)))
                                           (send x :args))))
                         (send (send client :get-result) :sequence)))
      (when debug
        (pprint (car (send (send client :get-result) :data))))
      (setq ret (parse-ff-result-string (car (send (send client :get-result) :data))))
      (setq ret (append ret (list (cons :plan plan)))))
    ret))

(defun solve-pddl-planning (domain problem &key (client *action-client*) (debug)
                                   (optimize-state nil))
  (let ((result (pddl-planning domain problem :client client :debug debug))
        cond-list)

    (unless optimize-state
      (let ((def-init (send problem :initial-condition))
            (sol-init (result-condition->problem-condition (cdr (assoc :initial result))))
            adds)
        (setq adds (set-difference def-init sol-init :test #'equal))
        (setq adds (problem-condition->result-condition adds))
        (setq result (add-conditions-to-result result adds))))

    (setq cond-list (apply-plan (cdr (assoc :initial result))
                                (cdr (assoc :plan result))
                                (cdr (assoc :action-list result))))
    (push (cons :step-state cond-list) result)
    result))
