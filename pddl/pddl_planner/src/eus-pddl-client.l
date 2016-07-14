(require :eus-pddl "package://pddl_planner/src/eus-pddl.l")

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
      (cond
       ((send (send client :get-result) :use_durative_action) ;; use durative action
        (setq plan (mapcar #'(lambda (x)
                               (cons (read-from-string (send x :start_time))
                                     (cons (read-from-string (send x :action_duration))
                                           (cons (read-from-string (send x :action))
                                                 (mapcar #'(lambda (y) (read-from-string y))
                                                         (send x :args))))))
                               (send (send client :get-result) :sequence)))
        )
       (t ;; normal action
        (setq plan (mapcar #'(lambda (x)
                               (cons (read-from-string (remove #\( (send x :action)))
                                     (mapcar #'(lambda (y) (read-from-string (remove #\) y)))
                                             (send x :args))))
                           (send (send client :get-result) :sequence)))
        ))
      (when debug
        (pprint (car (send (send client :get-result) :data))))
      (setq ret (append ret (list (cons :plan plan)))))
    ret))

(defun solve-pddl-planning (domain problem &key (client *action-client*) (debug)
                                   (optimize-state nil))
  (let* ((result (pddl-planning domain problem :client client :debug debug))
         (state (instance pddl-state :init :domain domain :initial-state (send problem :initial-condition) :actions (cdr (assoc :plan result)) :objects (send problem :objects)))
         (step-state (send state :generate-states)))

    (when optimize-state
      (let ((intersect-condition (reduce #'(lambda(x y) (intersection x y :test #'equal)) step-state)))
        (setq step-state (mapcar #'(lambda(x) (set-differece x intersect-condition)) step-state))))

    (when (send domain :use-durative-action)
      ;; reset plan to the timeline-actions
      (setq result (list (cons :plan  (send state :timeline-actions))))
      (push (cons :durative t) result)
      )
    (push (cons :step-state step-state) result)
    (push (cons :state state) result)
    result))

(provide :eus-pddl-client)
