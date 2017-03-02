#!/usr/bin/env roseus
;; execute-pddl.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "execute_pddl")
(require :execute-pddl-core "package://task_compiler/euslisp/execute-pddl-core.l")

(defmacro load-param (sym pname &optional required defval eval)
  `(progn
     (setq ,sym (ros::get-param ,pname ,defval))
     (when (and ,eval (stringp ,sym))
       (setq ,sym (read-from-string ,sym)))
     (when (and ,required (null ,sym))
       (ros::ros-error "param ~A must be set" ,pname)
       (exit 1))
     (ros::ros-info "~A: ~A" ,pname ,sym)
     ',sym))

(load-param *action-file* "~action_file" t)
(load-param *description-file* "~description_file" t)
(load-param *pddl-debug* "~debug" nil nil)
(load-param *return-success* "~return_success" nil :success t)
(load-param *return-fail* "~return_fail" nil :fail t)
(load-param *self-return-failure* "~self_return_failre" nil t)
(load-param *display-graph* "~display_graph" nil nil)
(load-param *iterate* "~iterate" nil nil)

(defun y-or-n-p (&optional format-string &rest args)
  (let (response)
    (tagbody
     retry
       (when format-string (apply #'format t format-string args))
       (format t "(Y or N): ")
       (finish-output t)
       (setq response (read-line t))
       (if (= (length response) 0) (go retry))
       (case (char-upcase (aref response 0))
         (#\Y (return-from y-or-n-p t))
         (#\N (return-from y-or-n-p nil))
         (t (go retry))))))

(setq *result*
      (run-pddl :action *action-file*
                :description *description-file*
                :return-success *return-success*
                :return-fail *return-fail*
                :display-graph *display-graph*
                :iterate *iterate*
                :add-self-return-failure-path *self-return-failure*))

(if *result* (ros::exit 0) (ros::exit 1))
