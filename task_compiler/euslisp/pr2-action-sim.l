;; this file is originally created by
;;   (output-domain-template "pr2-action.l" *domain*)

;; package generate
(let ((ns (string-upcase "pr2_action")))
  (when (and ns (not (find-package ns)))
      (make-package ns)))

(in-package "X")
(defclass testPanel :super panel
  :slots (success fail val))
(defmethod testPanel
  (:create (&rest args)
	   (send-super* :create :width 210 :height 180
			:font font-courb12 args)
	   (send-super :create-item button-item "success"
		       self :success :font font-courb14)
	   (send-super :create-item button-item "fail"
		       self :fail :font font-courb14)
	   (send-super :create-item text-item ""
		       self :label :font font-courb14)
	   self)
  (:label (obj) t)
  (:success (obj) (setq val t))
  (:fail    (obj) (setq val nil))
  (:val     (&rest args) (if args (setq val (car args)) val))
  )
(in-package "USER")
(setq *ui-panel* (instance x::testPanel :create))

;; action definition
(defun PR2_ACTION::pick (obj)
  (send *ui-panel* :val 0)
  (format t ";; pick action [~a] -> " obj)
  (finish-output t)
  (while (eq 0 (send *ui-panel* :val))
    (unix::usleep 100000)
    (x::window-main-one))
  (format t "~a~%" (if (send *ui-panel* :val) "success" "fail"))
  (send *ui-panel* :val))

(defun PR2_ACTION::put (obj base)
  (send *ui-panel* :val 0)
  (format t ";; put action [~a,~a] -> " obj base)
  (finish-output t)
  (while (eq 0 (send *ui-panel* :val))
    (unix::usleep 100000)
    (x::window-main-one))
  (format t "~a~%" (if (send *ui-panel* :val) "success" "fail"))
  (send *ui-panel* :val))