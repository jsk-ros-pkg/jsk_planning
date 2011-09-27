;; this file is originally created by
;;   (output-domain-template "pr2-action.l" *domain*)

;; package generate
(let ((ns (string-upcase "pr2_action")))
  (when (and ns (not (find-package ns)))
      (make-package ns)))

;; action definition
(defun PR2_ACTION::pick (obj)
  (let ((val (random 2)))
    (format t ";; pick action [~a] -> ~a~%"
	    obj (if (= 0 val) "fail" "success"))
    (if (= 0 val) nil t)))

(defun PR2_ACTION::put (obj base)
  (let ((val (random 2)))
    (format t ";; put action [~a ~a] -> ~a~%"
	    obj base (if (= 0 val) "fail" "success"))
    (if (= 0 val) nil t))))
