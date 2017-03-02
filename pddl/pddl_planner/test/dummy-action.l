;; dummy-action.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(defun move-to (spot)
  (ros::publish "move_to"
                (instance std_msgs::String :init
                          :data (format nil "(move-to ~A)" spot)))
  t)

(defun look (obj)
  (ros::publish "look"
                (instance std_msgs::String :init
                          :data (format nil "(look ~A)" obj)))
  (setq *look-success?* (null *look-success?*))
  (if *look-success?*
      t :fail))

(defun push-button (obj)
  (ros::publish "push_button"
                (instance std_msgs::String :init
                          :data (format nil "(push-button ~A)" obj)))
  (setq *push-success?* (null *push-success?*))
  (if *push-success?* t :fail))

(defun task-compiler/after-init-hook ()
  ;; this function is called when task_compiler is initialized
  (ros::ros-info "after_init_hook is called")
  (ros::set-param "after_init_hook" "called")
  (ros::advertise "move_to" std_msgs::String 1)
  (ros::advertise "look" std_msgs::String 1)
  (ros::advertise "push_button" std_msgs::String 1))

(defun task-compiler/before-run-hook (sm)
  ;; this function is called every time before planned task is executed
  (ros::ros-info "before_run_hook is called")
  (ros::set-param "before_run_hook" "called")
  (setq *look-success?* t)
  (setq *push-success?* t))

(defun task-compiler/after-run-hook (sm)
  ;; this function is called every time after planned task is executed
  (ros::ros-info "after_run_hook is called")
  (ros::set-param "after_run_hook" "called"))


(provide :dummy-action) ;; end of dummy-action.l
