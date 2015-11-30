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
    (ros::spin-once)
    (ros::sleep))
  (warn "~% (length *msg*) is ~A" (length *msg*))
  (assert (eq (length *msg*) 7)))

(ros::roseus "test_task_compiler")
(run-all-tests)
(exit)

(provide :test-task-compiler) ;; end of test-task-compiler.l
