;; test-task-compiler.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(defmacro subscribe-and-count (name)
  `(ros::subscribe ,name std_msgs::String #'(lambda (m)
                                              (format *error-output* "received ~A" (send m :data))
                                              (push m *msg*))))

(deftest test-task-compiler
  (setq *msg* nil)
  (subscribe-and-count "move_to")
  (subscribe-and-count "look")
  (subscribe-and-count "push")
  (setq *start* (ros::time-now))
  (ros::rate 1)
  (dotimes (i 30)
    (when (eq (length *msg*) 7)
      (return))
    (ros::ros-info "waiting task finish ~A / 7 (timeout: ~A / 30)" (length *msg*) i)
    (ros::spin-once)
    (ros::sleep))
  (warn "~% (length *msg*) is ~A" (length *msg*))
  (assert (eq (length *msg*) 7)))

(defun wait-param (key &optional (timeout 30))
  (dotimes (i timeout)
    (when (and (ros::has-param key))
      (ros::ros-info "found param ~A" key)
      (return-from wait-param t))
    (ros::ros-info "waiting param ~A [~A/~A]" key i timeout)
    (ros::spin-once)
    (ros::sleep))
  nil)

(deftest test-task-compiler-hook ()
  (ros::rate 1)
  (assert (wait-param "after_init_hook"))
  (assert (wait-param "before_run_hook"))
  (assert (wait-param "after_run_hook")))

(ros::roseus "test_task_compiler")
(run-all-tests)
(exit)

(provide :test-task-compiler) ;; end of test-task-compiler.l
